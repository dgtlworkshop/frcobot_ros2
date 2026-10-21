worker-threads := "2"

_default:
	@just --list

# Installs the project dependencies
install: _rosdep_install

_rosdep_install:
	rosdep install -i --from-path . -y

# Builds the workspace
build:
	colcon build --parallel-workers {{ worker-threads }}

# Builds the workspace
debug:
	colcon build --mixin debug --parallel-workers {{ worker-threads }}


# Builds the workspace for the purpose of generating intellisense
completions:
	colcon build --parallel-workers {{ worker-threads }} --continue-on-error --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Cleans the build folders
clean:
	rm -rf build install log

# Formats all code
format: _format_3_9_9

[working-directory("fairino_hardware_v3_9_9")]
_format_3_9_9:
	clang-format -i --style=file --verbose **/*.cpp include/**/*.h include/**/*.hpp
