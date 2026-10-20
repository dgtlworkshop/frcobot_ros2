workers := 4

_default:
	@just --list

# Installs the project dependencies
install: _rosdep_install

_rosdep_install:
	rosdep install -i --from-path . -y

# Builds the workspace
build:
	colcon build --symlink-install --parallel-workers {{ workers }}

# Builds the workspace
debug:
	colcon build --mixin debug --symlink-install --parallel-workers {{ workers }}


# Builds the workspace for the purpose of generating intellisense
completions:
	colcon build --symlink-install --continue-on-error --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers {{ workers }}

# Cleans the build folders
clean:
	rm -rf build install log

# Formats all code
format: _format_cpp

_format_cpp:
	clang-format -i --style=file --verbose **/*.cpp */include/**/*.h */include/**/*.hpp


# From https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html#install-ros-2-and-colcon
# Installs colcon extensions for debugging
[linux]
install-colcon-extensions:
	sudo apt-get install python3-colcon-common-extensions python3-colcon-mixin
	colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
	colcon mixin update default
