package main

import (
	"context"

	kinematicsutils "github.com/randhid/kinematics-utils"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	ctx := context.Background()
	logger := logging.NewLogger("cli")

	deps := resource.Dependencies{}
	// can load these from a remote machine if you need

	cfg := kinematicsutils.KinConfig{
		KinematicsFile: "ur20.json",
		CADFile:        "ur20.ply",
	}

	kincheck, err := kinematicsutils.NewKinematicsChecker(
		ctx, deps, arm.Named("foo"), &cfg, logger)
	if err != nil {
		return err
	}
	kin, err := kincheck.Kinematics(ctx)
	logger.Infof("kincheck: %v, err %v", kin, err)
	geoms, err := kincheck.Geometries(ctx, nil)
	logger.Infof("geoms: %v, err %v", geoms, err)
	defer kincheck.Close(ctx)

	meshviz, err := kinematicsutils.NewMeshViz(
		ctx, deps, gripper.Named("foo3"), &kinematicsutils.MeshVizConfig{
			MeshFile: "ur3e.ply",
		}, logger)
	if err != nil {
		return err
	}

	geoms, err = meshviz.Geometries(ctx, nil)
	logger.Infof("meshviz: %v, err %v", geoms, err)
	defer meshviz.Close(ctx)

	pcviz, err := kinematicsutils.NewPointCloudViz(
		ctx, deps, camera.Named("foo4"), &kinematicsutils.PointCloudVizConfig{
			PointCloudFile: "example.pcd",
		}, logger)
	if err != nil {
		return err
	}

	defer pcviz.Close(ctx)

	urdfcvrt, err := kinematicsutils.NewURDFConverter(
		ctx, deps, generic.Named("foo2"), &kinematicsutils.URDFConverterConfig{
			URDFFile: "ur20.urdf",
		}, logger)
	if err != nil {
		return err
	}
	urdf, err := urdfcvrt.DoCommand(ctx, map[string]interface{}{
		"urdf2sva": true,
	})
	logger.Infof("urdf: %v, err %v", urdf, err)
	defer urdfcvrt.Close(ctx)

	return nil
}
