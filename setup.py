import setuptools
import os

version = '3.5.2'

with open(os.path.join(os.path.dirname(__file__), 'README.md')) as f:
    LongDescription = f.read()

setuptools.setup(
    name='swoop-dronekit',
    zip_safe=True,
    version=version,
    description='Developer Tools for Drones.',
    long_description_content_type="text/markdown",
    long_description=LongDescription,
    url='https://github.com/dronekit/dronekit-python',
    author='3D Robotics',
    install_requires=[
        'monotonic>=1.3',
        'swoop-MAVProxy==3.5.1',
    ],
    author_email='tim@3drobotics.com, kevinh@geeksville.com',
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Environment :: Console',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering',
    ],
    license='apache',
    packages=setuptools.find_packages()
)
