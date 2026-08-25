from setuptools import setup

package_name = "nevo_gsheet_logger"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rifqy Fachrizi",
    maintainer_email="kiki@diagonal-robotics.com",
    description="NEVO Google Sheets Logger Node",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nevo_gsheet_node = nevo_gsheet_logger.nevo_gsheet_node:main",
        ],
    },
)
