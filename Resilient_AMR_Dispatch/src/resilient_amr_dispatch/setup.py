from glob import glob

from setuptools import find_packages, setup


package_name = "resilient_amr_dispatch"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test", "tests")),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jchiu",
    maintainer_email="jchiu@example.com",
    description="Resilient warehouse AMR dispatch simulation.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "amr_agent = resilient_amr_dispatch.amr_agent:main",
            "dispatch_node = resilient_amr_dispatch.dispatch_node:main",
        ],
    },
)
