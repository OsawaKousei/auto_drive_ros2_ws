import sys

import rclpy
from path_creator_widget import PathCreatorWidget
from path_publisher import PathPublisher
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)
from spline_interpolation_widget import SplineInterpolationWidget
from trapezoidal_velocity_widget import TrapezoidalVelocityWidget


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("自動運転経路作成ツール")
        self.setGeometry(100, 100, 900, 700)

        # ROS2ノードの初期化
        rclpy.init(args=None)
        self.path_publisher = PathPublisher()

        # ウィジェットの初期化
        self.initUI()

    def initUI(self) -> None:
        # メインウィジェットとスタックウィジェット
        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        main_layout = QVBoxLayout()
        main_widget.setLayout(main_layout)

        # スタックウィジェットで画面を切り替え
        self.stacked_widget = QStackedWidget()
        main_layout.addWidget(self.stacked_widget)

        # モード選択画面
        self.mode_selection_widget = self.create_mode_selection_widget()
        self.stacked_widget.addWidget(self.mode_selection_widget)

        # 各機能画面
        self.path_creator_widget = PathCreatorWidget(self.path_publisher)
        self.spline_widget = SplineInterpolationWidget(self.path_publisher)
        self.velocity_widget = TrapezoidalVelocityWidget(self.path_publisher)

        self.stacked_widget.addWidget(self.path_creator_widget)
        self.stacked_widget.addWidget(self.spline_widget)
        self.stacked_widget.addWidget(self.velocity_widget)

        # 戻るボタンの接続
        self.path_creator_widget.back_button.clicked.connect(self.show_mode_selection)
        self.spline_widget.back_button.clicked.connect(self.show_mode_selection)
        self.velocity_widget.back_button.clicked.connect(self.show_mode_selection)

    def create_mode_selection_widget(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout()
        widget.setLayout(layout)

        # タイトル
        title_label = QLabel("機能を選択してください")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 20px; font-weight: bold; margin: 20px;")
        layout.addWidget(title_label)

        # モード選択ボタン
        path_creator_button = QPushButton("パスを作成")
        path_creator_button.setMinimumHeight(50)
        path_creator_button.clicked.connect(
            lambda: self.stacked_widget.setCurrentWidget(self.path_creator_widget)
        )
        layout.addWidget(path_creator_button)

        spline_button = QPushButton("スプライン補間")
        spline_button.setMinimumHeight(50)
        spline_button.clicked.connect(
            lambda: self.stacked_widget.setCurrentWidget(self.spline_widget)
        )
        layout.addWidget(spline_button)

        velocity_button = QPushButton("コーナーに基づいた台形加減速")
        velocity_button.setMinimumHeight(50)
        velocity_button.clicked.connect(
            lambda: self.stacked_widget.setCurrentWidget(self.velocity_widget)
        )
        layout.addWidget(velocity_button)

        layout.addStretch()

        return widget

    def show_mode_selection(self) -> None:
        self.stacked_widget.setCurrentWidget(self.mode_selection_widget)

    def closeEvent(self, event: QCloseEvent) -> None:
        # ROS2ノードを終了
        self.path_publisher.destroy_node()
        rclpy.shutdown()
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
