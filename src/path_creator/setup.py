# setuptoolsのfind_packages関数、setup関数を使えるようにする
# osパッケージに含まれる関数群を使用可能にする
import os

# globパッケージからglob関数を使用可能にする
from glob import glob

from setuptools import find_packages, setup

package_name = 'path_creator'

setup(
    # パッケージ名を指定
    name=package_name,
    # パッケージのバージョンを指定
    version='0.0.0',
    # pythonのパッケージディレクトリを指定、testはテストコードを入れておくディレクトリなので除外する。
    packages=find_packages(exclude=['test']),
    data_files=[
        # 追加データなどを入れるリソースディレクトリを指定
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xmlというパッケージの依存を管理するファイルをインストール
        ('share/' + package_name, ['package.xml']),
    ],
    # setuptoolsを使ってこのパッケージをインストールすることを指定
    install_requires=['setuptools'],
    zip_safe=True,
    # パッケージのメンテナ（動かないときに連絡窓口になるひと）の名前
    maintainer='n622',
    # メンテナーの連絡先
    maintainer_email='n622jwith@gamil.com',
    # パッケージの説明
    description='Path Creator for ROS2 Humble',
    # パッケージのライセンスを指定
    license='Apache-2.0',
    # 単体テストのため依存を追加
    tests_require=['pytest'],
    # ros2 runコマンドやros2 launchコマンドでノードを起動できる王にするための設定。
    # ここを忘れていると実行ができません。
    entry_points={
        'console_scripts': [
            'path_creator_node = path_creator.path_creator.main_window:main',
        ],
    },
)
