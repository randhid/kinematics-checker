package kinematicsutils

import (
	"context"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var PointCloudViz = family.WithModel("pointcloud-viz")

type PointCloudVizConfig struct {
	PointCloudFile string           `json:"pointcloud_file"`
	Transform      spatialmath.Pose `json:"transform,omitempty"`
}

func (cfg *PointCloudVizConfig) Validate(path string) ([]string, []string, error) {
	if cfg.PointCloudFile == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "pointcloud_file")
	}
	return nil, nil, nil
}

type pointCloudViz struct {
	resource.Named
	resource.AlwaysRebuild
	cfg *PointCloudVizConfig

	pc pointcloud.PointCloud
}

func newPointCloudViz(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	conf, err := resource.NativeConfig[*PointCloudVizConfig](config)
	if err != nil {
		return nil, err
	}

	return NewPointCloudViz(ctx, deps, config.ResourceName(), conf, logger)
}

func NewPointCloudViz(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *PointCloudVizConfig, logger logging.Logger) (camera.Camera, error) {
	pc, err := pointcloud.NewFromFile(conf.PointCloudFile, pointcloud.BasicType)
	if err != nil {
		return nil, err
	}
	if conf.Transform != nil {
		oc, err := pointcloud.ToBasicOctree(pc, 0)
		if err != nil {
			return nil, err
		}
		oc.Transform(conf.Transform)
		pc = oc
	}

	pcv := &pointCloudViz{
		cfg: conf,
		pc:  pc,
	}
	return pcv, nil
}

func (pcv *pointCloudViz) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	return nil, camera.ImageMetadata{}, errUnimplemented
}

func (pcv *pointCloudViz) Images(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	return nil, resource.ResponseMetadata{}, errUnimplemented
}

func (pcv *pointCloudViz) NextPointCloud(ctx context.Context) (pointcloud.PointCloud, error) {
	return pcv.pc, nil
}

func (pcv *pointCloudViz) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{}, nil
}

func (pcv *pointCloudViz) Close(ctx context.Context) error {
	return nil
}

func (pcv *pointCloudViz) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, errUnimplemented
}
