from setuptools import setup, find_packages

setup(
    name="MuJoCo2Rai",
    version="0.0.1",
    packages=find_packages(),
    install_requires=[
        # TODO Add dependencies here, e.g., 'requests', 'numpy'
    ],
    entry_points={
        'console_scripts': [
            # Example: 'your_command = your_package.module:function'
        ]
    },
    author="Marc Toussaint, Denis Shcherba, Eckar Cobo Briesewitz",
    author_email="d.e.shcherba@gmail.com",
    description="A short description of your package",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/denis-shcherba/master_thesis",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.10',)
