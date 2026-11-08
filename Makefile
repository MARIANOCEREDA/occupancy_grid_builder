# Root Makefile for the entire workspace
# This Makefile coordinates building C++ libraries and ROS2 packages

ROS_DISTRO   ?= jazzy
ROS_SETUP     = /opt/ros/$(ROS_DISTRO)/setup.bash

# Workspace paths
WS_ROOT      := $(realpath $(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

# Camera-Lidar Fusion paths
CAMERA_LIDAR_FUSION_SRC    := $(WS_ROOT)/src/pcl_labeler
PCL_LABELER     := $(CAMERA_LIDAR_FUSION_SRC)/cpp/build

# Occupancy Grid Builder paths
OCCUPANCY_GRID_BUILDER_SRC := $(WS_ROOT)/src/occupancy_grid_builder
OCCUPANCY_GRID_BUILDER_CPP  := $(OCCUPANCY_GRID_BUILDER_SRC)/cpp/build

# YOLO ONNX ROS paths
YOLO_ONNX_ROS_SRC := $(WS_ROOT)/src/yolo_onnx_ros
YOLO_ONNX_CPP    := $(YOLO_ONNX_ROS_SRC)/src/yolo_onnx_ros/cpp/build

# Build settings
JOBS         ?= $(shell nproc)
BUILD_TYPE   ?= Release

.PHONY: help \
        build-pcl-labeler build-pcl-labeler-cpp build-pcl-labeler-ros2 \
        build-occupancy-grid-builder build-occupancy-grid-builder-cpp build-occupancy-grid-builder-ros2 \
        build-yolo-onnx-ros build-yolo-onnx-ros-cpp build-yolo-onnx-ros-ros2 \
        build-all-cpp build-all-ros2 build-all \
        clean-pcl-labeler clean-pcl-labeler-cpp clean-pcl-labeler-ros2 \
        clean-occupancy-grid-builder clean-occupancy-grid-builder-cpp clean-occupancy-grid-builder-ros2 \
        clean-yolo-onnx-ros clean-yolo-onnx-ros-cpp clean-yolo-onnx-ros-ros2 \
        clean-all-cpp clean-all-ros2 clean-all \
		install-rosdeps clone-gitmodules build-image-processing-ros2

help:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║                    Workspace Build System                                  ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@echo ""
	@echo "Component Build Targets:"
	@echo "  build-pcl-labeler          Build pcl-labeler (cpp + ros2)"
	@echo "  build-pcl-labeler-cpp      Build pcl-labeler C++ library only"
	@echo "  build-pcl-labeler-ros2     Build pcl-labeler ROS2 wrapper only"
	@echo ""
	@echo "  build-occupancy-grid-builder       Build occupancy-grid-builder (cpp + ros2)"
	@echo "  build-occupancy-grid-builder-cpp   Build occupancy-grid-builder C++ library only"
	@echo "  build-occupancy-grid-builder-ros2  Build occupancy-grid-builder ROS2 wrapper only"
	@echo ""
	@echo ""
	@echo "Aggregate Build Targets:"
	@echo "  build-all-cpp                      Build all C++ libraries"
	@echo "  build-all-ros2                     Build all ROS2 packages"
	@echo "  build-all                          Build everything (cpp + ros2)"
	@echo ""
	@echo "Clean Targets:"
	@echo "  clean-pcl-labeler          Clean pcl-labeler build artifacts"
	@echo "  clean-pcl-labeler-cpp      Clean pcl-labeler C++ library"
	@echo "  clean-pcl-labeler-ros2     Clean pcl-labeler ROS2 build"
	@echo ""
	@echo "  clean-occupancy-grid-builder       Clean occupancy-grid-builder build artifacts"
	@echo "  clean-occupancy-grid-builder-cpp   Clean occupancy-grid-builder C++ library"
	@echo "  clean-occupancy-grid-builder-ros2  Clean occupancy-grid-builder ROS2 build"
	@echo ""
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

build-image-processing-ros2:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building image-processing ROS2 package                                    ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@bash -c "source $(ROS_SETUP) && \
		cd $(WS_ROOT) && \
		colcon build \
			--symlink-install \
			--parallel-workers $(JOBS) \
			--packages-select image_processing \
			--event-handlers console_cohesion+"
	@echo " image-processing ROS2 package built successfully"
	@echo ""


build-pcl-labeler-cpp:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building pcl-labeler C++ library                                  ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@mkdir -p $(PCL_LABELER)
	@cd $(PCL_LABELER) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DCMAKE_INSTALL_PREFIX=$(PCL_LABELER)/install && \
		make -j$(JOBS) && \
		make install
	@echo " pcl-labeler C++ library built successfully"
	@echo ""

build-pcl-labeler-ros2: build-pcl-labeler-cpp
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building pcl-labeler ROS2 wrapper                                 ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@bash -c "source $(ROS_SETUP) && \
		cd $(WS_ROOT) && \
		colcon build \
			--symlink-install \
			--parallel-workers $(JOBS) \
			--packages-select pcl_labeler \
			--cmake-args -DPCL_LABELER=$(PCL_LABELER) \
			--event-handlers console_cohesion+"
	@echo " pcl-labeler ROS2 wrapper built successfully"
	@echo ""

build-pcl-labeler: build-pcl-labeler-ros2

clean-pcl-labeler-cpp:
	@echo "==> Cleaning pcl-labeler C++ library..."
	@rm -rf $(PCL_LABELER)
	@echo " Done"

clean-pcl-labeler-ros2:
	@echo "==> Cleaning pcl-labeler ROS2 build..."
	@rm -rf $(WS_ROOT)/build/pcl_labeler
	@rm -rf $(WS_ROOT)/install/pcl_labeler
	@echo " Done"

clean-pcl-labeler: clean-pcl-labeler-cpp clean-pcl-labeler-ros2

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
	@echo " occupancy-grid-builder C++ library built successfully"
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
	@echo " occupancy-grid-builder ROS2 wrapper built successfully"
	@echo ""

build-occupancy-grid-builder: build-occupancy-grid-builder-ros2

clean-occupancy-grid-builder-cpp:
	@echo "==> Cleaning occupancy-grid-builder C++ library..."
	@rm -rf $(OCCUPANCY_GRID_BUILDER_CPP)
	@echo " Done"

clean-occupancy-grid-builder-ros2:
	@echo "==> Cleaning occupancy-grid-builder ROS2 build..."
	@rm -rf $(WS_ROOT)/build/occupancy_grid_builder
	@rm -rf $(WS_ROOT)/install/occupancy_grid_builder
	@echo " Done"

clean-occupancy-grid-builder: clean-occupancy-grid-builder-cpp clean-occupancy-grid-builder-ros2

# ============================================================================
# YOLO ONNX ROS
# ============================================================================

build-yolo-onnx-ros-cpp:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building yolo-onnx-ros C++ library                                        ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@$(MAKE) -C $(YOLO_ONNX_ROS_SRC) build-cpp JOBS=$(JOBS)
	@echo " yolo-onnx-ros C++ library built successfully"
	@echo ""

build-yolo-onnx-ros-ros2: build-yolo-onnx-ros-cpp
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Building yolo-onnx-ros ROS2 wrapper                                       ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@bash -c "source $(ROS_SETUP) && \
		cd $(WS_ROOT) && \
		colcon build \
			--symlink-install \
			--parallel-workers $(JOBS) \
			--packages-select yolo_onnx_ros \
			--cmake-args -DYOLO_CPP_PREFIX=$(YOLO_ONNX_CPP) \
			--event-handlers console_cohesion+"
	@echo " yolo-onnx-ros ROS2 wrapper built successfully"
	@echo ""

build-yolo-onnx-ros: build-yolo-onnx-ros-ros2

clean-yolo-onnx-ros-cpp:
	@echo "==> Cleaning yolo-onnx-ros C++ library..."
	@rm -rf $(YOLO_ONNX_CPP)
	@echo " Done"

clean-yolo-onnx-ros-ros2:
	@echo "==> Cleaning yolo-onnx-ros ROS2 build..."
	@rm -rf $(WS_ROOT)/build/yolo_onnx_ros
	@rm -rf $(WS_ROOT)/install/yolo_onnx_ros
	@echo " Done"

clean-yolo-onnx-ros: clean-yolo-onnx-ros-cpp clean-yolo-onnx-ros-ros2

# ============================================================================
# Aggregate Targets
# ============================================================================

build-all-cpp: build-pcl-labeler-cpp build-occupancy-grid-builder-cpp build-yolo-onnx-ros-cpp
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
				-DPCL_LABELER=$(PCL_LABELER) \
				-DOCCUPANCY_GRID_BUILDER_CPP=$(OCCUPANCY_GRID_BUILDER_CPP) \
				-DYOLO_CPP_PREFIX=$(YOLO_ONNX_CPP) \
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

clean-all-cpp: clean-pcl-labeler-cpp clean-occupancy-grid-builder-cpp clean-yolo-onnx-ros-cpp
	@echo " All C++ libraries cleaned"

clean-all-ros2:
	@echo "==> Cleaning all ROS2 build artifacts..."
	@rm -rf $(WS_ROOT)/build $(WS_ROOT)/install $(WS_ROOT)/log
	@echo " All ROS2 builds cleaned"

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
	@echo " ROS2 dependencies installed"
	@echo ""

clone-gitmodules:
	@echo ""
	@echo "╔════════════════════════════════════════════════════════════════════════════╗"
	@echo "║  Initializing and updating git submodules                                   ║"
	@echo "╚════════════════════════════════════════════════════════════════════════════╝"
	@bash -c "cd $(WS_ROOT) && \
		git submodule update --init --recursive"
	@echo " Git submodules cloned"
	@echo ""
