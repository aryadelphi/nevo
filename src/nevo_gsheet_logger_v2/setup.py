from setuptools import setup

package_name = "nevo_gsheet_logger_v2"

setup(
    name=package_name,
    version="2.0.0",
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
    description="NEVO Google Sheets Logger v2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nevo_gsheet_node = nevo_gsheet_logger_v2.nevo_gsheet_node:main",
        ],
    },
)
