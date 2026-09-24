from glob import glob
from setuptools import setup


package_name = "lynsense_webots_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=["lynsense_webots_sim", "lynsense_webots_sim.scripts"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/lynsense_webots_sim"],
        ),
        ("share/lynsense_webots_sim/config", glob("config/*")),
        ("share/lynsense_webots_sim/worlds", glob("worlds/*")),
        ("share/lynsense_webots_sim/trees", glob("trees/*")),
        ("share/lynsense_webots_sim/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RPent",
    maintainer_email="maintainer@rpent.local",
    description="Webots smoke-test navigation package for Lynsense.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lynsense_webots_controller = lynsense_webots_sim.webots_controller:main",
            "lynsense_action_probe = lynsense_webots_sim.scripts.action_probe:main",
            "lynsense_run_smoke = lynsense_webots_sim.scripts.run_smoke:main",
            "lynsense_run_viewer = lynsense_webots_sim.scripts.run_viewer:main",
            "lynsense_single_box_tree = lynsense_webots_sim.scripts.run_single_box_tree:main",
            "lynsense_run_rpent_plan = lynsense_webots_sim.scripts.run_rpent_plan:main",
        ]
    },
)
