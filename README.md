# car_AI_RL

# 功能

用於將Unity的資料透過Rosbridge傳輸到RL訓練環境進行訓練
目前都還在thread_testing測試
# 各個程式介紹

* AI_node.py
  * 收送資料和取得經轉換後的資料
* UnityAdaptor.py
  * 轉換資料，和決定訓練資料輸出
* car_dataCollect.py
  * 已經廢棄使用
* car_testing.py
  * 測試Unity資料收送和測試UnityAdaptor轉換是否正常
* env.py
  * 用gym的模板做的訓練環境，step、reset、reward相關功能都在這邊
* main.py
  * 主程式碼
* tools.py
  * reward計算會使用到的工具

# 測試用資料夾(用於測試其他方法)

* senior_sister (學姊的程式碼)
* AINode_ver (修改node運作)

# 執行程式碼

1. 啟動Unity (目前因傳輸問題，所以有順序關係)
2. 執行`python3 main.py`

# 待改善問題(解決了)

* Unity之間的資料傳輸可能因為太快導致資料遺失或是其他問題，造成此端的訓練程式無法接收到最新的state
* 經過第一次reset後，step輸出action會出現異常(一直做重複的動作都不變)和傳輸過來的資料不是最新的state導致無法正常reset

# 修改方式

* 裡面隨機生成位置有邏輯上問題，車體與目標有機率生成在一樣的地方已修復
* lidar的程式碼改成輸出360個角度，同時讓lidar的射線可視化(變得比較炫炮)
* 傳輸問題，每次做完動作後會延遲0.7秒才會送出，python的RL做過處理，所以一定會讀到最新的資料
# 待改善問題

* Unity之間的資料傳輸可能因為太快導致資料遺失或是其他問題，造成此端的訓練程式無法接收到最新的state
* 經過第一次reset後，step輸出action會出現異常(一直做重複的動作都不變)和傳輸過來的資料不是最新的state導致無法正常reset
