from path_publisher import PathPublisher
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class SplineInterpolationWidget(QWidget):
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
        title_label = QLabel("スプライン補間")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        main_layout.addLayout(header_layout)

        # 設定グループ
        settings_group = QGroupBox("補間設定")
        settings_layout = QVBoxLayout()
        settings_group.setLayout(settings_layout)

        # 補間点数設定
        points_layout = QHBoxLayout()
        points_layout.addWidget(QLabel("補間点数:"))
        self.points_spinbox = QSpinBox()
        self.points_spinbox.setRange(10, 1000)
        self.points_spinbox.setValue(100)
        points_layout.addWidget(self.points_spinbox)
        points_layout.addStretch()
        settings_layout.addLayout(points_layout)

        # 平滑化パラメータ
        smooth_layout = QHBoxLayout()
        smooth_layout.addWidget(QLabel("平滑化パラメータ:"))
        self.smooth_spinbox = QDoubleSpinBox()
        self.smooth_spinbox.setRange(0.0, 1.0)
        self.smooth_spinbox.setSingleStep(0.1)
        self.smooth_spinbox.setValue(0.5)
        smooth_layout.addWidget(self.smooth_spinbox)
        smooth_layout.addStretch()
        settings_layout.addLayout(smooth_layout)

        # 補間実行ボタン
        interpolate_button = QPushButton("スプライン補間を実行")
        interpolate_button.clicked.connect(self.perform_interpolation)
        settings_layout.addWidget(interpolate_button)

        main_layout.addWidget(settings_group)

        # 結果表示エリア
        result_group = QGroupBox("補間結果")
        result_layout = QVBoxLayout()
        result_group.setLayout(result_layout)

        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setPlainText(
            "スプライン補間機能は開発中です。\n\n"
            "この機能では、経路点間をスプライン曲線で補間し、\n"
            "滑らかな軌道を生成します。"
        )
        result_layout.addWidget(self.result_text)

        main_layout.addWidget(result_group)

    def perform_interpolation(self) -> None:
        # TODO: スプライン補間のロジックを実装
        points = self.points_spinbox.value()
        smooth = self.smooth_spinbox.value()

        self.result_text.setPlainText(
            f"スプライン補間設定:\n"
            f"補間点数: {points}\n"
            f"平滑化パラメータ: {smooth}\n\n"
            f"補間処理の実装が必要です。"
        )
