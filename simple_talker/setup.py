from setuptools import setup

package_name = "simple_talker"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    py_modules=[
        "simple_talker.simple_talker_string",
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shunsugar",
    maintainer_email="shun.saito.6t@stu.hosei.ac.jp",
    description="hoge",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simple_talker_string = simple_talker.simple_talker_string:main",
        ],
    },
)
