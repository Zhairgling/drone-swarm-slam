"""Verify the ground_station launch description loads without errors."""
import importlib.util
import pathlib


def test_launch_description_loads():
    """Load the main launch file and verify it returns a LaunchDescription."""
    from launch import LaunchDescription  # noqa: PLC0415

    launch_file = (
        pathlib.Path(__file__).parent.parent / "launch" / "ground_station.launch.py"
    )
    assert launch_file.exists(), f"Launch file not found: {launch_file}"

    spec = importlib.util.spec_from_file_location("ground_station_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    ld = module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)


def test_launch_description_declares_num_drones_arg():
    """Check the launch description declares the num_drones argument."""
    from launch.actions import DeclareLaunchArgument  # noqa: PLC0415

    launch_file = (
        pathlib.Path(__file__).parent.parent / "launch" / "ground_station.launch.py"
    )
    spec = importlib.util.spec_from_file_location("ground_station_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    ld = module.generate_launch_description()
    arg_names = [
        e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)
    ]
    assert "num_drones" in arg_names, (
        f"Expected 'num_drones' launch argument, got: {arg_names}"
    )
