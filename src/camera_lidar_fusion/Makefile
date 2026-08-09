ROS_DISTRO   ?= jazzy
ROS_SETUP     = /opt/ros/$(ROS_DISTRO)/setup.bash

REPO_ROOT    := $(realpath $(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
WS_ROOT      := $(REPO_ROOT)/../..
CAMERA_LIDAR_FUSION_CPP_DIR := $(REPO_ROOT)/cpp
CAMERA_LIDAR_FUSION_CPP := $(CAMERA_LIDAR_FUSION_CPP_DIR)/build

JOBS         ?= $(shell nproc)

.PHONY: help build-cpp-library build-ros2 build-all clean-cpp clean-ros2 clean

help:
	@echo ""
	@echo "Available targets:"
	@echo "  build-cpp-library  Build the mask_lidar_fusion C++ library only (CMake)"
	@echo "  build-ros2         Build all ROS2 packages (colcon)"
	@echo "  build-all          Build C++ library then ROS2 packages"
	@echo "  clean-cpp          Remove the C++ library build directory"
	@echo "  clean-ros2         Remove colcon build/install/log directories"
	@echo "  clean              Remove all build artifacts"
	@echo "  install-rosdeps    Install ROS2 dependencies"
	@echo ""
	@echo "Options (override with make <target> OPTION=value):"
	@echo "  ROS_DISTRO=<distro>   ROS2 distribution name  (default: jazzy)"
	@echo ""

build-cpp-library:
	@echo "==> Building mask_lidar_fusion C++ library..."
	@mkdir -p $(CAMERA_LIDAR_FUSION_CPP)
	@cd $(CAMERA_LIDAR_FUSION_CPP) && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(CAMERA_LIDAR_FUSION_CPP)/install && make -j$(JOBS) && make install
	@echo "==> C++ library build complete."

clean-cpp:
	@echo "==> Cleaning C++ library build..."
	@rm -rf $(CAMERA_LIDAR_FUSION_CPP)
	@echo "==> Done."

build-ros2:
	@echo "==> Building ROS2 packages from workspace: $(WS_ROOT)"
	@bash -c "source $(ROS_SETUP) && \
	          cd $(WS_ROOT) && \
	          colcon build \
	            --symlink-install \
	            --parallel-workers $(JOBS) \
	            --packages-select camera_lidar_fusion \
	            --cmake-args -DCAMERA_LIDAR_FUSION_CPP=$(CAMERA_LIDAR_FUSION_CPP) \
	            --event-handlers console_cohesion+"
	@echo "==> ROS2 build complete."

clean-ros2:
	@echo "==> Cleaning ROS2 build artifacts in $(WS_ROOT)..."
	@rm -rf $(WS_ROOT)/build $(WS_ROOT)/install $(WS_ROOT)/log
	@echo "==> Done."

build-all:
	@echo "==> Building all components..."
	@$(MAKE) build-cpp-library
	@$(MAKE) build-ros2
	@echo "==> All components built successfully."

install-rosdeps:
	@echo "==> Installing ROS2 dependencies..."
	@bash -c "source $(ROS_SETUP) && \
	          cd $(WS_ROOT) && \
			  sudo apt update && \
	          rosdep update && \
	          rosdep install --from-paths src --ignore-src -r -y"
	@echo "==> ROS2 dependencies installed."

clean:
	@$(MAKE) clean-cpp
	@$(MAKE) clean-ros2