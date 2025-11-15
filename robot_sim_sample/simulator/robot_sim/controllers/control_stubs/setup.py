import os
from glob import glob
from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

package_name = 'control_stubs'
proto_files = glob(os.path.join(package_name, '*.proto'))


class BuildProtobuf(build_py):
    def run(self):
        # 生成 gRPC 代码
        if not self.dry_run:
            for proto_file in proto_files:
                self.announce(f'Generating gRPC code from {proto_file}', level=3)
                os.system(
                    'python3 -m grpc_tools.protoc '
                    '-I. '  # 搜索 proto 文件的路径
                    '--python_out=. '  # 输出 Python 文件的路径
                    '--grpc_python_out=. '  # 输出 gRPC Python 文件的路径
                    '--pyi_out=. '  # 类型提示输出目录
                    f'{proto_file}'
                )
        super().run()  # 继续标准构建流程


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        package_name: ["*.py", "*.pyi", "*.proto"],
    },
    install_requires=["setuptools", "grpcio", "grpcio-tools"],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    cmdclass={'build_py': BuildProtobuf},  # 覆盖标准构建命令
    entry_points={
        'console_scripts': [
        ],
    },
)
