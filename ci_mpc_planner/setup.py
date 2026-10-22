from setuptools import setup

package_name = "ci_mpc_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nils Johnson",
    maintainer_email="dr.nils1@gmail.com",
    description="Stage-1 MPC for the FR3 (end-effector trajectory QP on top of the impedance controller)",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["mpc_node = ci_mpc_planner.mpc_node:main"]},
)
