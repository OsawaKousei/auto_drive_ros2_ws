import csv
import sys

import rclpy
from path_publisher import PathPublisher
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QCloseEvent, QDoubleValidator
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("経路作成ツール")
        self.setGeometry(100, 100, 800, 600)

        # ROS2ノードの初期化
        rclpy.init(args=None)
        self.path_publisher = PathPublisher()

        # ウィジェットの初期化
        self.initUI()

    def initUI(self) -> None:
        # メインウィジェットとレイアウト
        main_widget = QWidget()
        main_layout = QVBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

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

        # Z座標
        z_layout = QHBoxLayout()
        z_layout.addWidget(QLabel("Z:"))
        self.z_input = QLineEdit()
        self.z_input.setValidator(QDoubleValidator())
        self.z_input.setText("0.0")  # デフォルト値
        z_layout.addWidget(self.z_input)
        coord_layout.addLayout(z_layout)

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

        self.waypoint_table = QTableWidget(0, 4)  # 行数0, 列数4 (ID, X, Y, Z)
        self.waypoint_table.setHorizontalHeaderLabels(["ID", "X", "Y", "Z"])
        self.waypoint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.waypoint_table.setSelectionBehavior(QTableWidget.SelectRows)
        # セルが編集されたときのシグナルを接続
        self.waypoint_table.cellChanged.connect(self.on_cell_changed)
        table_layout.addWidget(self.waypoint_table)

        # テーブル操作ボタン
        table_buttons_layout = QHBoxLayout()

        # 上へ移動ボタン
        up_button = QPushButton("↑ 上へ")
        up_button.clicked.connect(self.move_up)
        table_buttons_layout.addWidget(up_button)

        # 下へ移動ボタン
        down_button = QPushButton("↓ 下へ")
        down_button.clicked.connect(self.move_down)
        table_buttons_layout.addWidget(down_button)

        # 削除ボタン
        delete_button = QPushButton("削除")
        delete_button.clicked.connect(self.delete_waypoint)
        table_buttons_layout.addWidget(delete_button)

        table_layout.addLayout(table_buttons_layout)
        main_layout.addWidget(table_group)

        # ファイル操作ボタングループ
        file_buttons_layout = QHBoxLayout()

        # CSVインポートボタン
        import_button = QPushButton("CSVからインポート")
        import_button.clicked.connect(self.import_from_csv)
        file_buttons_layout.addWidget(import_button)

        # CSVエクスポートボタン
        export_button = QPushButton("CSVにエクスポート")
        export_button.clicked.connect(self.export_to_csv)
        file_buttons_layout.addWidget(export_button)

        main_layout.addLayout(file_buttons_layout)

    def add_waypoint(self) -> None:
        try:
            x = float(self.x_input.text() or 0.0)
            y = float(self.y_input.text() or 0.0)
            z = float(self.z_input.text() or 0.0)

            row_position = self.waypoint_table.rowCount()
            self.waypoint_table.insertRow(row_position)

            # IDは1始まり
            # IDセルを編集不可に設定
            id_item = QTableWidgetItem(str(row_position + 1))
            id_item.setFlags(id_item.flags() & ~Qt.ItemIsEditable)
            self.waypoint_table.setItem(row_position, 0, id_item)
            self.waypoint_table.setItem(row_position, 1, QTableWidgetItem(str(x)))
            self.waypoint_table.setItem(row_position, 2, QTableWidgetItem(str(y)))
            self.waypoint_table.setItem(row_position, 3, QTableWidgetItem(str(z)))

            # 入力欄をクリア
            self.x_input.clear()
            self.y_input.clear()

            # パスを更新
            self.update_path()

        except ValueError:
            QMessageBox.warning(self, "入力エラー", "数値を正しく入力してください")

    def on_cell_changed(self, row: int, column: int) -> None:
        # IDカラム (column=0) の変更は無視
        if column > 0:
            # 一時的にcellChangedシグナルの接続を切断して再帰を防止
            self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)

            try:
                # 変更されたセルの内容が数値かチェック
                text = self.waypoint_table.item(row, column).text()
                value = float(text)
                # 有効な数値なら、パスを更新
                self.update_path()
            except ValueError:
                # 数値でない場合はエラーメッセージを表示し、値を0.0にリセット
                QMessageBox.warning(self, "入力エラー", "数値を正しく入力してください")
                self.waypoint_table.setItem(row, column, QTableWidgetItem("0.0"))
            finally:
                # シグナルの接続を復活
                self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def import_from_csv(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self, "CSVファイルから読み込み", "", "CSV Files (*.csv);;All Files (*)"
        )

        if filename:
            try:
                # 既存の経路点をクリア
                self.waypoint_table.setRowCount(0)
                self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)

                with open(filename, 'r', newline='') as file:
                    reader = csv.reader(file)
                    header = next(reader)  # ヘッダー行をスキップ

                    for row_data in reader:
                        if len(row_data) >= 4:  # ID, X, Y, Zの4列あることを確認
                            row_position = self.waypoint_table.rowCount()
                            self.waypoint_table.insertRow(row_position)

                            # IDセルを編集不可に設定
                            id_item = QTableWidgetItem(row_data[0])
                            id_item.setFlags(id_item.flags() & ~Qt.ItemIsEditable)
                            self.waypoint_table.setItem(row_position, 0, id_item)

                            # X, Y, Z値を設定
                            for col in range(1, 4):
                                if col < len(row_data):
                                    try:
                                        # 数値の検証
                                        value = float(row_data[col])
                                        self.waypoint_table.setItem(
                                            row_position,
                                            col,
                                            QTableWidgetItem(str(value)),
                                        )
                                    except ValueError:
                                        # 数値でない場合は0.0を設定
                                        self.waypoint_table.setItem(
                                            row_position, col, QTableWidgetItem("0.0")
                                        )

                # パスを更新
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
                # シグナルの接続を復活
                self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def move_up(self) -> None:
        selected_row = self.waypoint_table.currentRow()
        self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)

        if selected_row > 0:
            # 行を入れ替え
            for col in range(1, 4):  # X, Y, Z列のみ入れ替える
                current = self.waypoint_table.item(selected_row, col).text()
                above = self.waypoint_table.item(selected_row - 1, col).text()

                self.waypoint_table.setItem(
                    selected_row - 1, col, QTableWidgetItem(current)
                )
                self.waypoint_table.setItem(selected_row, col, QTableWidgetItem(above))

            # 選択行を更新
            self.waypoint_table.setCurrentCell(selected_row - 1, 0)

            # パスを更新
            self.update_path()

        self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def move_down(self) -> None:
        selected_row = self.waypoint_table.currentRow()
        self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)

        if selected_row >= 0 and selected_row < self.waypoint_table.rowCount() - 1:
            # 行を入れ替え
            for col in range(1, 4):  # X, Y, Z列のみ入れ替える
                current = self.waypoint_table.item(selected_row, col).text()
                below = self.waypoint_table.item(selected_row + 1, col).text()

                self.waypoint_table.setItem(
                    selected_row + 1, col, QTableWidgetItem(current)
                )
                self.waypoint_table.setItem(selected_row, col, QTableWidgetItem(below))

            # 選択行を更新
            self.waypoint_table.setCurrentCell(selected_row + 1, 0)

            # パスを更新
            self.update_path()

        self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def delete_waypoint(self) -> None:
        selected_row = self.waypoint_table.currentRow()
        self.waypoint_table.cellChanged.disconnect(self.on_cell_changed)

        if selected_row >= 0:
            self.waypoint_table.removeRow(selected_row)

            # IDを振り直す
            for row in range(self.waypoint_table.rowCount()):
                id_item = QTableWidgetItem(str(row + 1))
                id_item.setFlags(id_item.flags() & ~Qt.ItemIsEditable)
                self.waypoint_table.setItem(row, 0, id_item)

            # パスを更新
            self.update_path()

        self.waypoint_table.cellChanged.connect(self.on_cell_changed)

    def update_path(self) -> None:
        waypoints = []
        for row in range(self.waypoint_table.rowCount()):
            x = float(self.waypoint_table.item(row, 1).text())
            y = float(self.waypoint_table.item(row, 2).text())
            z = float(self.waypoint_table.item(row, 3).text())
            waypoints.append((x, y, z))

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
                    writer.writerow(["ID", "X", "Y", "Z"])

                    for row in range(self.waypoint_table.rowCount()):
                        id_val = self.waypoint_table.item(row, 0).text()
                        x = self.waypoint_table.item(row, 1).text()
                        y = self.waypoint_table.item(row, 2).text()
                        z = self.waypoint_table.item(row, 3).text()
                        writer.writerow([id_val, x, y, z])

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
