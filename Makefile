# Root Makefile for the entire workspace
# This Makefile coordinates building C++ libraries and ROS2 packages

ROS_DISTRO   ?= jazzy
ROS_SETUP     = /opt/ros/$(ROS_DISTRO)/setup.bash

# Workspace paths
WS_ROOT      := $(realpath $(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

# Camera-Lidar Fusion paths
CAMERA_LIDAR_FUSION_SRC    := $(WS_ROOT)/src/camera_lidar_fusion
CAMERA_LIDAR_FUSION_CPP     := $(CAMERA_LIDAR_FUSION_SRC)/cpp/build

# Occupancy Grid Builder paths
OCCUPANCY_GRID_BUILDER_SRC := $(WS_ROOT)/src/occupancy_grid_builder
OCCUPANCY_GRID_BUILDER_CPP  := $(OCCUPANCY_GRID_BUILDER_SRC)/cpp/build

# Segmentation Inference ONNX paths
SEGMENTATION_INFERENCE_ONNX_SRC := $(WS_ROOT)/src/segmentation_inference_onnx
SEGMENTATION_INFERENCE_ONNX_CPP := $(SEGMENTATION_INFERENCE_ONNX_SRC)/cpp/build
SEGMENTATION_INFERENCE_ONNX_PREFIX := $(SEGMENTATION_INFERENCE_ONNX_CPP)/install

# Build settings
JOBS         ?= $(shell nproc)
BUILD_TYPE   ?= Release

.PHONY: help \
        build-camera-lidar-fusion build-camera-lidar-fusion-cpp build-camera-lidar-fusion-ros2 \
        build-occupancy-grid-builder build-occupancy-grid-builder-cpp build-occupancy-grid-builder-ros2 \
		build-segmentation-inference-onnx build-segmentation-inference-onnx-cpp build-segmentation-inference-onnx-ros2 \
        build-all-cpp build-all-ros2 build-all \
        clean-camera-lidar-fusion clean-camera-lidar-fusion-cpp clean-camera-lidar-fusion-ros2 \
        clean-occupancy-grid-builder clean-occupancy-grid-builder-cpp clean-occupancy-grid-builder-ros2 \
		clean-segmentation-inference-onnx clean-segmentation-inference-onnx-cpp clean-segmentation-inference-onnx-ros2 \
        clean-all-cpp clean-all-ros2 clean-all \
        install-rosdeps

help:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║                    Workspace Build System                                  ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@echo ""
	@echo "Component Build Targets:"
	@echo "  build-camera-lidar-fusion          Build camera-lidar-fusion (cpp + ros2)"
	@echo "  build-camera-lidar-fusion-cpp      Build camera-lidar-fusion C++ library only"
	@echo "  build-camera-lidar-fusion-ros2     Build camera-lidar-fusion ROS2 wrapper only"
	@echo ""
	@echo "  build-occupancy-grid-builder       Build occupancy-grid-builder (cpp + ros2)"
	@echo "  build-occupancy-grid-builder-cpp   Build occupancy-grid-builder C++ library only"
	@echo "  build-occupancy-grid-builder-ros2  Build occupancy-grid-builder ROS2 wrapper only"
	@echo ""
	@echo "  build-segmentation-inference-onnx       Build segmentation-inference-onnx (cpp + ros2)"
	@echo "  build-segmentation-inference-onnx-cpp   Build segmentation-inference-onnx C++ library only"
	@echo "  build-segmentation-inference-onnx-ros2  Build segmentation-inference-onnx ROS2 wrapper only"
	@echo ""
	@echo "Aggregate Build Targets:"
	@echo "  build-all-cpp                      Build all C++ libraries"
	@echo "  build-all-ros2                     Build all ROS2 packages"
	@echo "  build-all                          Build everything (cpp + ros2)"
	@echo ""
	@echo "Clean Targets:"
	@echo "  clean-camera-lidar-fusion          Clean camera-lidar-fusion build artifacts"
	@echo "  clean-camera-lidar-fusion-cpp      Clean camera-lidar-fusion C++ library"
	@echo "  clean-camera-lidar-fusion-ros2     Clean camera-lidar-fusion ROS2 build"
	@echo ""
	@echo "  clean-occupancy-grid-builder       Clean occupancy-grid-builder build artifacts"
	@echo "  clean-occupancy-grid-builder-cpp   Clean occupancy-grid-builder C++ library"
	@echo "  clean-occupancy-grid-builder-ros2  Clean occupancy-grid-builder ROS2 build"
	@echo ""
	@echo "  clean-segmentation-inference-onnx       Clean segmentation-inference-onnx build artifacts"
	@echo "  clean-segmentation-inference-onnx-cpp   Clean segmentation-inference-onnx C++ library"
	@echo "  clean-segmentation-inference-onnx-ros2  Clean segmentation-inference-onnx ROS2 build"
	@echo ""
	@echo "  clean-all-cpp                      Clean all C++ library builds"
	@echo "  clean-all-ros2                     Clean all ROS2 builds"
	@echo "  clean-all                          Clean everything"
	@echo ""
	@echo "Utility Targets:"
	@echo "  install-rosdeps                    Install ROS2 dependencies"
	@echo ""
	@echo "Options (override with make <target> OPTION=value):"
	@echo "  ROS_DISTRO=<distro>                ROS2 distribution (default: jazzy)"
	@echo "  JOBS=<n>                           Parallel jobs (default: nproc)"
	@echo "  BUILD_TYPE=<type>                  CMake build type (default: Release)"
	@echo ""

# ============================================================================
# Camera-Lidar Fusion
# ============================================================================

build-camera-lidar-fusion-cpp:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building camera-lidar-fusion C++ library                                  ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@mkdir -p $(CAMERA_LIDAR_FUSION_CPP)
	@cd $(CAMERA_LIDAR_FUSION_CPP) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DCMAKE_INSTALL_PREFIX=$(CAMERA_LIDAR_FUSION_CPP)/install && \
		make -j$(JOBS) && \
		make install
	@echo "✓ camera-lidar-fusion C++ library built successfully"
	@echo ""

build-camera-lidar-fusion-ros2: build-camera-lidar-fusion-cpp
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building camera-lidar-fusion ROS2 wrapper                                 ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@bash -c "source $(ROS_SETUP) && \
		cd $(WS_ROOT) && \
		colcon build \
			--symlink-install \
			--parallel-workers $(JOBS) \
			--packages-select camera_lidar_fusion \
			--cmake-args -DCAMERA_LIDAR_FUSION_CPP=$(CAMERA_LIDAR_FUSION_CPP) \
			--event-handlers console_cohesion+"
	@echo "✓ camera-lidar-fusion ROS2 wrapper built successfully"
	@echo ""

build-camera-lidar-fusion: build-camera-lidar-fusion-ros2

clean-camera-lidar-fusion-cpp:
	@echo "==> Cleaning camera-lidar-fusion C++ library..."
	@rm -rf $(CAMERA_LIDAR_FUSION_CPP)
	@echo "✓ Done"

clean-camera-lidar-fusion-ros2:
	@echo "==> Cleaning camera-lidar-fusion ROS2 build..."
	@rm -rf $(WS_ROOT)/build/camera_lidar_fusion
	@rm -rf $(WS_ROOT)/install/camera_lidar_fusion
	@echo "✓ Done"

clean-camera-lidar-fusion: clean-camera-lidar-fusion-cpp clean-camera-lidar-fusion-ros2

# ============================================================================
# Occupancy Grid Builder
# ============================================================================

build-occupancy-grid-builder-cpp:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building occupancy-grid-builder C++ library                               ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@mkdir -p $(OCCUPANCY_GRID_BUILDER_CPP)
	@cd $(OCCUPANCY_GRID_BUILDER_CPP) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DCMAKE_INSTALL_PREFIX=$(OCCUPANCY_GRID_BUILDER_CPP)/install && \
		make -j$(JOBS) && \
		make install
	@echo "✓ occupancy-grid-builder C++ library built successfully"
	@echo ""

build-occupancy-grid-builder-ros2: build-occupancy-grid-builder-cpp
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building occupancy-grid-builder ROS2 wrapper                              ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@bash -c "source $(ROS_SETUP) && \
		cd $(WS_ROOT) && \
		colcon build \
			--symlink-install \
			--parallel-workers $(JOBS) \
			--packages-select occupancy_grid_builder \
			--cmake-args -DOCCUPANCY_GRID_BUILDER_CPP=$(OCCUPANCY_GRID_BUILDER_CPP) \
			--event-handlers console_cohesion+"
	@echo "✓ occupancy-grid-builder ROS2 wrapper built successfully"
	@echo ""

build-occupancy-grid-builder: build-occupancy-grid-builder-ros2

clean-occupancy-grid-builder-cpp:
	@echo "==> Cleaning occupancy-grid-builder C++ library..."
	@rm -rf $(OCCUPANCY_GRID_BUILDER_CPP)
	@echo "✓ Done"

clean-occupancy-grid-builder-ros2:
	@echo "==> Cleaning occupancy-grid-builder ROS2 build..."
	@rm -rf $(WS_ROOT)/build/occupancy_grid_builder
	@rm -rf $(WS_ROOT)/install/occupancy_grid_builder
	@echo "✓ Done"

clean-occupancy-grid-builder: clean-occupancy-grid-builder-cpp clean-occupancy-grid-builder-ros2

# ============================================================================
# Segmentation Inference ONNX C++
# ============================================================================

build-segmentation-inference-onnx-cpp:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building segmentation-inference-onnx C++ library                           ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@mkdir -p $(SEGMENTATION_INFERENCE_ONNX_CPP)
	@cd $(SEGMENTATION_INFERENCE_ONNX_CPP) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DCMAKE_INSTALL_PREFIX=$(SEGMENTATION_INFERENCE_ONNX_PREFIX) && \
		make -j$(JOBS) && \
		make install
	@echo "✓ segmentation-inference-onnx C++ library built successfully"
	@echo ""

build-segmentation-inference-onnx-ros2: build-segmentation-inference-onnx-cpp
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building segmentation-inference-onnx ROS2 wrapper                         ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@bash -c "source $(ROS_SETUP) && \
		cd $(WS_ROOT) && \
		colcon build \
			--symlink-install \
			--parallel-workers $(JOBS) \
			--packages-select segmentation_inference_onnx_ros \
			--cmake-args -DYOLO_CPP_PREFIX=$(SEGMENTATION_INFERENCE_ONNX_PREFIX) \
			--event-handlers console_cohesion+"
	@echo "✓ segmentation-inference-onnx ROS2 wrapper built successfully"
	@echo ""

build-segmentation-inference-onnx: build-segmentation-inference-onnx-ros2

clean-segmentation-inference-onnx-cpp:
	@echo "==> Cleaning segmentation-inference-onnx C++ library..."
	@rm -rf $(SEGMENTATION_INFERENCE_ONNX_CPP)
	@echo "✓ Done"

clean-segmentation-inference-onnx-ros2:
	@echo "==> Cleaning segmentation-inference-onnx ROS2 build..."
	@rm -rf $(WS_ROOT)/build/segmentation_inference_onnx_ros
	@rm -rf $(WS_ROOT)/install/segmentation_inference_onnx_ros
	@echo "✓ Done"

clean-segmentation-inference-onnx: clean-segmentation-inference-onnx-cpp clean-segmentation-inference-onnx-ros2

# ============================================================================
# Aggregate Targets
# ============================================================================

build-all-cpp: build-camera-lidar-fusion-cpp build-occupancy-grid-builder-cpp build-segmentation-inference-onnx-cpp
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  All C++ libraries built successfully                                      ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@echo ""

build-all-ros2: build-all-cpp
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building all ROS2 packages                                                ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@bash -c "source $(ROS_SETUP) && \
		cd $(WS_ROOT) && \
		colcon build \
			--symlink-install \
			--parallel-workers $(JOBS) \
			--cmake-args \
				-DCAMERA_LIDAR_FUSION_CPP=$(CAMERA_LIDAR_FUSION_CPP) \
				-DOCCUPANCY_GRID_BUILDER_CPP=$(OCCUPANCY_GRID_BUILDER_CPP) \
				-DYOLO_CPP_PREFIX=$(SEGMENTATION_INFERENCE_ONNX_PREFIX) \
			--event-handlers console_cohesion+"
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  All ROS2 packages built successfully                                      ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@echo ""

build-all: build-all-ros2
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Complete workspace built successfully!                                    ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@echo ""
	@bash -c "source $(WS_ROOT)/install/setup.bash"

clean-all-cpp: clean-camera-lidar-fusion-cpp clean-occupancy-grid-builder-cpp clean-segmentation-inference-onnx-cpp
	@echo "✓ All C++ libraries cleaned"

clean-all-ros2:
	@echo "==> Cleaning all ROS2 build artifacts..."
	@rm -rf $(WS_ROOT)/build $(WS_ROOT)/install $(WS_ROOT)/log
	@echo "✓ All ROS2 builds cleaned"

clean-all: clean-all-cpp clean-all-ros2
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Workspace cleaned                                                         ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@echo ""

install-rosdeps:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Installing ROS2 dependencies                                              ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@bash -c "source $(ROS_SETUP) && \
		cd $(WS_ROOT) && \
		sudo apt update && \
		rosdep update && \
		rosdep install --from-paths src --ignore-src -r -y"
	@echo "✓ ROS2 dependencies installed"
	@echo ""
