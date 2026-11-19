package main

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"

	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var g1IMUModel = resource.NewModel("erh", "viam-unitree", "g1-imu")

// G1IMUConfig configures a movement sensor over a standalone unitree_hg
// IMUState_ DDS topic. The default is the G1 torso IMU.
type G1IMUConfig struct {
	NetworkInterface string `json:"network_interface"`
	// Topic defaults to "rt/secondary_imu" (torso IMU). "rt/lf/secondary_imu"
	// is the same IMU at 20Hz.
	Topic string `json:"topic"`
}

func (c *G1IMUConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(movementsensor.API, g1IMUModel,
		resource.Registration[movementsensor.MovementSensor, *G1IMUConfig]{
			Constructor: newG1IMU,
		})
}

type g1IMU struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	imu    *IMUClient

	mu sync.Mutex
}

func newG1IMU(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (movementsensor.MovementSensor, error) {
	cfg, err := resource.NativeConfig[*G1IMUConfig](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}
	topic := "rt/secondary_imu"
	if cfg.Topic != "" {
		topic = cfg.Topic
	}

	logger.Infof("Initializing G1 IMU (interface=%s topic=%s)", networkInterface, topic)

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	imu, err := NewIMUClient(topic)
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("IMU subscribe: %w", err)
	}

	return &g1IMU{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		imu:    imu,
	}, nil
}

func (s *g1IMU) latest() (IMUState, error) {
	s.mu.Lock()
	imu := s.imu
	s.mu.Unlock()
	if imu == nil {
		return IMUState{}, fmt.Errorf("imu closed")
	}
	return imu.Latest()
}

func (s *g1IMU) Position(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
	return nil, 0, movementsensor.ErrMethodUnimplementedPosition
}

func (s *g1IMU) LinearVelocity(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	return r3.Vector{}, movementsensor.ErrMethodUnimplementedLinearVelocity
}

func (s *g1IMU) AngularVelocity(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
	st, err := s.latest()
	if err != nil {
		return spatialmath.AngularVelocity{}, err
	}
	// Viam expects deg/s; the IMU publishes rad/s.
	return spatialmath.AngularVelocity{
		X: float64(st.Gyro[0]) * 180.0 / math.Pi,
		Y: float64(st.Gyro[1]) * 180.0 / math.Pi,
		Z: float64(st.Gyro[2]) * 180.0 / math.Pi,
	}, nil
}

func (s *g1IMU) LinearAcceleration(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	st, err := s.latest()
	if err != nil {
		return r3.Vector{}, err
	}
	return r3.Vector{X: float64(st.Accel[0]), Y: float64(st.Accel[1]), Z: float64(st.Accel[2])}, nil
}

func (s *g1IMU) CompassHeading(ctx context.Context, extra map[string]interface{}) (float64, error) {
	return 0, movementsensor.ErrMethodUnimplementedCompassHeading
}

func (s *g1IMU) Orientation(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
	st, err := s.latest()
	if err != nil {
		return nil, err
	}
	// Unitree quaternion is w, x, y, z.
	return &spatialmath.Quaternion{
		Real: float64(st.Quaternion[0]),
		Imag: float64(st.Quaternion[1]),
		Jmag: float64(st.Quaternion[2]),
		Kmag: float64(st.Quaternion[3]),
	}, nil
}

func (s *g1IMU) Properties(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
	return &movementsensor.Properties{
		OrientationSupported:        true,
		AngularVelocitySupported:    true,
		LinearAccelerationSupported: true,
	}, nil
}

func (s *g1IMU) Accuracy(ctx context.Context, extra map[string]interface{}) (*movementsensor.Accuracy, error) {
	return movementsensor.UnimplementedOptionalAccuracies(), nil
}

func (s *g1IMU) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	readings, err := movementsensor.DefaultAPIReadings(ctx, s, extra)
	if err != nil {
		return nil, err
	}
	st, err := s.latest()
	if err != nil {
		return nil, err
	}
	s.mu.Lock()
	imu := s.imu
	s.mu.Unlock()

	readings["rpy_rad"] = map[string]interface{}{
		"roll":  float64(st.RPY[0]),
		"pitch": float64(st.RPY[1]),
		"yaw":   float64(st.RPY[2]),
	}
	readings["temperature_c"] = int(st.Temperature)
	readings["age_ms"] = float64(time.Since(st.ReceivedAt).Microseconds()) / 1000.0
	readings["source_time"] = st.SourceTime.UTC().Format(time.RFC3339Nano)
	// Transport delay plus the offset between the publisher's clock and this
	// host's. The G1's two computers aren't necessarily synced, so treat this
	// as a jitter/staleness signal, not an absolute latency.
	readings["receive_minus_source_ms"] = float64(st.ReceivedAt.Sub(st.SourceTime).Microseconds()) / 1000.0
	if imu != nil {
		readings["samples_taken"] = imu.Samples()
	}
	return readings, nil
}

func (s *g1IMU) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *g1IMU) Close(ctx context.Context) error {
	s.mu.Lock()
	imu := s.imu
	s.imu = nil
	s.mu.Unlock()

	if imu == nil {
		return nil // already closed; don't double-release the InitDDS ref
	}
	imu.Close()
	ShutdownDDS()
	return nil
}
