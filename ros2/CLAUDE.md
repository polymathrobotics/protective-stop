# ROS 2 conventions — always read and follow

**Canonical source (authoritative, read it):**
- ROS 2: https://github.com/polymathrobotics/polymath_core/blob/main/docs/core/style-guide/ros2-style-guide.md
- C++: https://github.com/polymathrobotics/polymath_core/blob/main/docs/core/style-guide/cpp-style-guide.md
- Launch migration: https://github.com/polymathrobotics/polymath_core/blob/main/docs/core/style-guide/launchfile-migration.md

(Rendered site `core-docs.polymathrobotics.dev/style-guide/` is behind Cloudflare Access SSO — fetch the raw markdown from the GitHub path above, e.g. `gh api repos/polymathrobotics/polymath_core/contents/docs/core/style-guide/ros2-style-guide.md --jq .content | base64 -d`.) When editing an external package, internal consistency beats this guide.

## Key rules (summary — the source is authoritative)

**Package metadata / build**
- Package names: clear and concise by what they do; avoid `_utils`/`_manager`; no "clever" thematic names.
- `package.xml`: schema tag for xmllint; version starts `0.0.0`; meaningful description; **maintainer = a team, not a person** (`engineering@polymathrobotics.com`, "Polymath Robotics Engineering"); **license `Proprietary` for internal code — NOT Apache** (only open-license when the code is opened; note: this repo, protective-stop, is a deliberate OSS exception at Apache-2.0). Dep types: `test/`→`test_depend`; python import→`exec_depend`; C++ include+link→`build_depend`/`depend`; codegen/CMake→`buildtool_depend`.
- **Ament CMake:** call `ament_*` functions as little as possible. Use `ament_cmake_auto` (`ament_auto_find_build_dependencies()`). **Prefer `target_link_libraries` over `ament_target_dependencies`** (fall back to the latter only for packages not exporting modern CMake targets; for msgs use `${pkg_TARGETS}`). C++17. Strict flags: `-Wall -Wextra -Wpedantic -Werror=switch` + `-Wl,--no-undefined`. Split into a core logic library + a thin `_nodelib`; `rclcpp_components_register_node(... EXECUTABLE ...)` to autogen the executable (no `main.cpp`) — **exception: lifecycle nodes that autostart need their own main** (e.g. this repo's machine node). Smoke-test nodes/launchfiles (`polymath_add_node_smoke_test`, `polymath_add_launch_smoke_test`).

**Interfaces**
- **Parameters via `generate_parameter_library`** from a YAML spec (`type`/`default_value`/`read_only`/`description`/`validation` e.g. `bounds<>: [lo,hi]`) — don't hand-roll declare+validate.
- **Semantic message types** — a message type should convey meaning (`safety_msgs/DetectedImpact`), not generic `Int32Stamped`/`std_msgs/String`. Block comments in `.msg/.srv/.action`, not inline.

**Node usage**
- **Thin nodes**: put logic in a standalone library with clear interfaces; the Node is just the runtime-invocation "main", not the business logic.
- Names via **remapping** (launch/CLI); behavior config via **Parameters**.
- "Callback" is a usage pattern, not a function name — name methods for what they do.
- **Launch files: YAML unless Python is absolutely necessary.** Prefer no launch file for a single node. If Python: never `PythonLaunchDescriptionSource`; always `PathSubstitution` (never `os.path.join`); always `FindPackageShare` (never `ament_index_python`); condensed single-`return` declarative style.

**Multi-distro**
- Monobranch: CI runs all supported distros. Set `ROS2_${DISTRO}` (via `polymath_cmakeutils`); write against the newest supported distro and shim older ones; split usage code only when unavoidable.

**Naming / leanness (as practiced — David Tarazi's `david1244-cleanup`, lead-endorsed):** descriptive names, no single-letter vars (`state`, not `s`); composable component; delete dead/redundant code and unused interfaces. See memory `feedback_ros2_conventions_david`; reference impl `ros2/protective_stop_machine`.
