from path_publisher import PathPublisher
from PyQt5.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class TrapezoidalVelocityWidget(QWidget):
    def __init__(self, path_publisher: PathPublisher) -> None:
        super().__init__()
        self.path_publisher = path_publisher
        self.initUI()

    def initUI(self) -> None:
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        # タイトルと戻るボタン
        header_layout = QHBoxLayout()
        self.back_button = QPushButton("← 戻る")
        header_layout.addWidget(self.back_button)
        header_layout.addStretch()
        title_label = QLabel("コーナーに基づいた台形加減速")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        main_layout.addLayout(header_layout)

        # 速度設定グループ
        velocity_group = QGroupBox("速度設定")
        velocity_layout = QVBoxLayout()
        velocity_group.setLayout(velocity_layout)

        # 最大速度
        max_vel_layout = QHBoxLayout()
        max_vel_layout.addWidget(QLabel("最大速度 [m/s]:"))
        self.max_velocity_spinbox = QDoubleSpinBox()
        self.max_velocity_spinbox.setRange(0.1, 10.0)
        self.max_velocity_spinbox.setSingleStep(0.1)
        self.max_velocity_spinbox.setValue(2.0)
        max_vel_layout.addWidget(self.max_velocity_spinbox)
        max_vel_layout.addStretch()
        velocity_layout.addLayout(max_vel_layout)

        # コーナー速度
        corner_vel_layout = QHBoxLayout()
        corner_vel_layout.addWidget(QLabel("コーナー速度 [m/s]:"))
        self.corner_velocity_spinbox = QDoubleSpinBox()
        self.corner_velocity_spinbox.setRange(0.1, 5.0)
        self.corner_velocity_spinbox.setSingleStep(0.1)
        self.corner_velocity_spinbox.setValue(0.5)
        corner_vel_layout.addWidget(self.corner_velocity_spinbox)
        corner_vel_layout.addStretch()
        velocity_layout.addLayout(corner_vel_layout)

        # 加速度設定
        accel_layout = QHBoxLayout()
        accel_layout.addWidget(QLabel("最大加速度 [m/s²]:"))
        self.max_acceleration_spinbox = QDoubleSpinBox()
        self.max_acceleration_spinbox.setRange(0.1, 5.0)
        self.max_acceleration_spinbox.setSingleStep(0.1)
        self.max_acceleration_spinbox.setValue(1.0)
        accel_layout.addWidget(self.max_acceleration_spinbox)
        accel_layout.addStretch()
        velocity_layout.addLayout(accel_layout)

        main_layout.addWidget(velocity_group)

        # コーナー検出設定
        corner_group = QGroupBox("コーナー検出設定")
        corner_layout = QVBoxLayout()
        corner_group.setLayout(corner_layout)

        # 角度閾値
        angle_layout = QHBoxLayout()
        angle_layout.addWidget(QLabel("角度閾値 [度]:"))
        self.angle_threshold_spinbox = QDoubleSpinBox()
        self.angle_threshold_spinbox.setRange(5.0, 180.0)
        self.angle_threshold_spinbox.setSingleStep(5.0)
        self.angle_threshold_spinbox.setValue(30.0)
        angle_layout.addWidget(self.angle_threshold_spinbox)
        angle_layout.addStretch()
        corner_layout.addLayout(angle_layout)

        # 自動検出オプション
        self.auto_detect_checkbox = QCheckBox("コーナーを自動検出")
        self.auto_detect_checkbox.setChecked(True)
        corner_layout.addWidget(self.auto_detect_checkbox)

        main_layout.addWidget(corner_group)

        # 実行ボタン
        generate_button = QPushButton("台形加減速プロファイルを生成")
        generate_button.clicked.connect(self.generate_velocity_profile)
        main_layout.addWidget(generate_button)

        # 結果表示エリア
        result_group = QGroupBox("生成結果")
        result_layout = QVBoxLayout()
        result_group.setLayout(result_layout)

        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setPlainText(
            "台形加減速機能は開発中です。\n\n"
            "この機能では、コーナーを検出し、\n"
            "台形加減速プロファイルを生成します。"
        )
        result_layout.addWidget(self.result_text)

        main_layout.addWidget(result_group)

    def generate_velocity_profile(self) -> None:
        # TODO: 台形加減速プロファイル生成のロジックを実装
        max_vel = self.max_velocity_spinbox.value()
        corner_vel = self.corner_velocity_spinbox.value()
        max_accel = self.max_acceleration_spinbox.value()
        angle_threshold = self.angle_threshold_spinbox.value()
        auto_detect = self.auto_detect_checkbox.isChecked()

        self.result_text.setPlainText(
            f"台形加減速設定:\n"
            f"最大速度: {max_vel} m/s\n"
            f"コーナー速度: {corner_vel} m/s\n"
            f"最大加速度: {max_accel} m/s²\n"
            f"角度閾値: {angle_threshold}°\n"
            f"自動検出: {'有効' if auto_detect else '無効'}\n\n"
            f"速度プロファイル生成の実装が必要です。"
        )
