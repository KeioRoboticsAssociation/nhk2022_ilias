# nhk2022_simulator
## これは何

nhk2022の環境をgazeboで再現するパッケージ。
今年度はメカナムなので特に物理シミュレーションをする意味が無いのでロジック確認のためのみ。


## ロボットモデル

現在は以下の2つに対応

- 簡易的な差動2輪モデル
- 簡易的な4輪オムニモデル (推奨)



## Launch

- フィールド+簡易的な4輪オムニモデル

  - R2のシミュレーターを起動
    ```
    roslaunch nhk2022_simulator simulation_R2.launch
    ```



- フィールド+簡易的な差動2輪モデル

  ```shell
  roslaunch nhk2022_simulator diff_drive_simulation.launch
  ```



上記launchを実行後

- R2

```shell
roslaunch nhk2022_launcher control_R2.launch
```

のロボット起動launchをたたくとロボットのシミュレーションが可能
