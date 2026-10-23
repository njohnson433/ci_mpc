from glob import glob

from setuptools import setup

package_name = "ci_mpc_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nils Johnson",
    maintainer_email="dr.nils1@gmail.com",
    description="Wrist camera perception: simulated Gemini 305 depth noise and a box detector",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["depth_noise = ci_mpc_perception.depth_noise:main",
                                      "box_detector = ci_mpc_perception.box_detector:main"]},
)
