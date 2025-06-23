package kinematicsutils

import (
	"context"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

// New creates a new URDFConverterService
func New(ctx context.Context) (*URDFConverterService, error) {
	return &URDFConverterService{}, nil
}

// DoCommand handles generic service commands
func (s *URDFConverterService) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

// Close implements the resource.Resource interface
func (s *URDFConverterService) Close(ctx context.Context) error {
	return nil
}

type URDFConverterConfig struct {
	URDFFile string `json:"urdf-file"`
}

// Validate ensures all parts of the config are valid and important fields exist.
func (cfg *URDFConverterConfig) Validate(path string) ([]string, []string, error) {
	if cfg.URDFFile == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "urdf-file")
	}
	return nil, nil, nil
}

type URDFConverterService struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	logger logging.Logger
	cfg    *URDFConverterConfig
}

// Name returns the name of the service
func (s *URDFConverterService) Name() resource.Name {
	return s.name
}

func newURDFConverter(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*URDFConverterConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewURDFConverter(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewURDFConverter(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *URDFConverterConfig, logger logging.Logger) (resource.Resource, error) {
	return &URDFConverterService{
		name:   name,
		logger: logger,
		cfg:    conf,
	}, nil
}
