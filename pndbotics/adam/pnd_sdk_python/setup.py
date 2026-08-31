import os
from setuptools import setup, find_packages

setup(name='pndbotics_sdk_py',
      version='1.0.1',
      author='PNDRobotics',
      author_email='peter.cui@pndbotics.com',
      long_description=open('README.md').read() if os.path.exists('README.md') else 'PND robot sdk version 2 for python',
      long_description_content_type="text/markdown",
      license="BSD-3-Clause",
      packages=find_packages(include=['pndbotics_sdk_py','pndbotics_sdk_py.*']),
      description='PND robot sdk version 2 for python',
      project_urls={
            "Source Code": "https://gitlab.pndbotics.com/pnd_group/pnd_sdk_python.git",
      },
      python_requires='>=3.8',
      install_requires=[
            "cyclonedds==0.10.2",
            "numpy",
            "opencv-python",
      ],
      )