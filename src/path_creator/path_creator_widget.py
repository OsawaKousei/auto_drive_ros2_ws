import csv

from path_publisher import PathPublisher
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtWidgets import (
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


class PathCreatorWidget(QWidget):
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
        title_label = QLabel("経路作成")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        main_layout.addLayout(header_layout)

        # 入力フォームグループ
        input_group = QGroupBox("経路点入力")
        input_layout = QVBoxLayout()
        input_group.setLayout(input_layout)

        # 座標入力
        coord_layout = QHBoxLayout()

        # X座標
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X:"))
        self.x_input = QLineEdit()
        self.x_input.setValidator(QDoubleValidator())
        x_layout.addWidget(self.x_input)
        coord_layout.addLayout(x_layout)

        # Y座標
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y:"))
        self.y_input = QLineEdit()
        self.y_input.setValidator(QDoubleValidator())
        y_layout.addWidget(self.y_input)
        coord_layout.addLayout(y_layout)

        # Yaw角入力
        yaw_layout = QHBoxLayout()
        yaw_layout.addWidget(QLabel("Yaw:"))
        self.yaw_input = QLineEdit()
        self.yaw_input.setValidator(QDoubleValidator())
        self.yaw_input.setText("0.0")
        yaw_layout.addWidget(self.yaw_input)
        coord_layout.addLayout(yaw_layout)

        input_layout.addLayout(coord_layout)

        # 追加ボタン
        add_button = QPushButton("経路点を追加")
        add_button.clicked.connect(self.add_waypoint)
        input_layout.addWidget(add_button)

        main_layout.addWidget(input_group)

        # 経路点テーブル
        table_group = QGroupBox("経路点リスト")
        table_layout = QVBoxLayout()
        table_group.setLayout(table_layout)

        self.waypoint_table = QTableWidget(0, 4)
        self.waypoint_table.setHorizontalHeaderLabels(["ID", "X", "Y", "Yaw"])
        self.waypoint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.waypoint_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.waypoint_table.cellChanged.connect(self.on_cell_changed)
        table_layout.addWidget(self.waypoint_table)

        # テーブル操作ボタン
        table_buttons_layout = QHBoxLayout()

        up_button = QPushButton("↑ 上へ")
        up_button.clicked.connect(self.move_up)
        table_buttons_layout.addWidget(up_button)

        down_button = QPushButton("↓ 下へ")
        down_button.clicked.connect(self.move_down)
        table_buttons_layout.addWidget(down_button)

        delete_button = QPushButton("削除")
        delete_button.clicked.connect(self.delete_waypoint)
        table_buttons_layout.addWidget(delete_button)

        table_layout.addLayout(table_buttons_layout)
        main_layout.addWidget(table_group)

        # ファイル操作ボタングループ
        file_buttons_layout = QHBoxLayout()

        import_button = QPushButton("CSVからインポート")
        import_button.clicked.connect(self.import_from_csv)
        file_buttons_layout.addWidget(import_button)

        export_button = QPushButton("CSVにエクスポート")
        export_button.clicked.connect(self.export_to_csv)
        file_buttons_layout.addWidget(export_button)

        main_layout.addLayout(file_buttons_layout)

    def add_waypoint(self) -> None:
        try:
            x = float(self.x_input.text() or 0.0)
            y = float(self.y_input.text() or 0.0)
            yaw = float(self.yaw_input.text() or 0.0)

            row_position = self.waypoint_table.rowCount()
            self.waypoint_table.insertRow(row_position)

            id_item = QTableWidgetItem(str(row_position + 1))
            id_item.setFlags(id_item.flags() & ~Qt.ItemIsEditable)
            self.waypoint_table.setItem(row_position, 0, id_item)
            self.waypoint_table.setItem(row_position, 1, QTableWidgetItem(str(x)))
            self.waypoint_table.setItem(row_position, 2, QTableWidgetItem(str(y)))
            self.waypoint_table.setItem(row_position, 3, QTableWidgetItem(str(yaw)))

            self.x_input.clear()
            self.y_input.clear()
            self.update_path()

        except ValueError:
            QMessageBox.warning(self, "入力エラー", "数値を正しく入力してください")

    def on_cell_changed(self, row: int, column: int) -> None:
        if column > 0:
            self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)
            try:
                text = self.waypoint_table.item(row, column).text()
                value = float(text)
                self.update_path()
            except ValueError:
                QMessageBox.warning(self, "入力エラー", "数値を正しく入力してください")
                self.waypoint_table.setItem(row, column, QTableWidgetItem("0.0"))
            finally:
                self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def import_from_csv(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self, "CSVファイルから読み込み", "", "CSV Files (*.csv);;All Files (*)"
        )

        if filename:
            try:
                self.waypoint_table.setRowCount(0)
                self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)

                with open(filename, 'r', newline='') as file:
                    reader = csv.reader(file)
                    header = next(reader)

                    for row_data in reader:
                        if len(row_data) >= 4:
                            row_position = self.waypoint_table.rowCount()
                            self.waypoint_table.insertRow(row_position)

                            id_item = QTableWidgetItem(row_data[0])
                            id_item.setFlags(id_item.flags() & ~Qt.ItemIsEditable)
                            self.waypoint_table.setItem(row_position, 0, id_item)

                            for col in range(1, 3):
                                if col < len(row_data):
                                    try:
                                        value = float(row_data[col])
                                        self.waypoint_table.setItem(
                                            row_position,
                                            col,
                                            QTableWidgetItem(str(value)),
                                        )
                                    except ValueError:
                                        self.waypoint_table.setItem(
                                            row_position, col, QTableWidgetItem("0.0")
                                        )

                            yaw = float(row_data[3])
                            self.waypoint_table.setItem(
                                row_position, 3, QTableWidgetItem(str(yaw))
                            )

                self.update_path()
                QMessageBox.information(
                    self,
                    "インポート成功",
                    f"CSVファイルから{self.waypoint_table.rowCount()}個の経路点を読み込みました。",
                )

            except Exception as e:
                QMessageBox.critical(
                    self,
                    "インポートエラー",
                    f"CSVの読み込み中にエラーが発生しました: {str(e)}",
                )
            finally:
                self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def move_up(self) -> None:
        selected_row = self.waypoint_table.currentRow()
        self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)

        if selected_row > 0:
            for col in range(1, 4):
                current = self.waypoint_table.item(selected_row, col).text()
                above = self.waypoint_table.item(selected_row - 1, col).text()

                self.waypoint_table.setItem(
                    selected_row - 1, col, QTableWidgetItem(current)
                )
                self.waypoint_table.setItem(selected_row, col, QTableWidgetItem(above))

            self.waypoint_table.setCurrentCell(selected_row - 1, 0)
            self.update_path()

        self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def move_down(self) -> None:
        selected_row = self.waypoint_table.currentRow()
        self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)

        if selected_row >= 0 and selected_row < self.waypoint_table.rowCount() - 1:
            for col in range(1, 4):
                current = self.waypoint_table.item(selected_row, col).text()
                below = self.waypoint_table.item(selected_row + 1, col).text()

                self.waypoint_table.setItem(
                    selected_row + 1, col, QTableWidgetItem(current)
                )
                self.waypoint_table.setItem(selected_row, col, QTableWidgetItem(below))

            self.waypoint_table.setCurrentCell(selected_row + 1, 0)
            self.update_path()

        self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def delete_waypoint(self) -> None:
        selected_row = self.waypoint_table.currentRow()
        self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)

        if selected_row >= 0:
            self.waypoint_table.removeRow(selected_row)

            for row in range(self.waypoint_table.rowCount()):
                id_item = QTableWidgetItem(str(row + 1))
                id_item.setFlags(id_item.flags() & ~Qt.ItemIsEditable)
                self.waypoint_table.setItem(row, 0, id_item)

            self.update_path()

        self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def update_path(self) -> None:
        waypoints = []
        for row in range(self.waypoint_table.rowCount()):
            x = float(self.waypoint_table.item(row, 1).text())
            y = float(self.waypoint_table.item(row, 2).text())
            yaw = float(self.waypoint_table.item(row, 3).text())
            waypoints.append((x, y, yaw))

        self.path_publisher.update_path(waypoints)

    def export_to_csv(self) -> None:
        if self.waypoint_table.rowCount() == 0:
            QMessageBox.warning(
                self, "エクスポートエラー", "経路点が設定されていません"
            )
            return

        filename, _ = QFileDialog.getSaveFileName(
            self, "CSVファイルに保存", "", "CSV Files (*.csv);;All Files (*)"
        )

        if filename:
            try:
                with open(filename, 'w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["ID", "X", "Y", "Yaw"])

                    for row in range(self.waypoint_table.rowCount()):
                        id_val = self.waypoint_table.item(row, 0).text()
                        x = self.waypoint_table.item(row, 1).text()
                        y = self.waypoint_table.item(row, 2).text()
                        yaw = self.waypoint_table.item(row, 3).text()
                        writer.writerow([id_val, x, y, yaw])

                QMessageBox.information(
                    self,
                    "エクスポート成功",
                    f"経路点をCSVファイルに保存しました: {filename}",
                )

            except Exception as e:
                QMessageBox.critical(
                    self,
                    "エクスポートエラー",
                    f"CSVの保存中にエラーが発生しました: {str(e)}",
                )
