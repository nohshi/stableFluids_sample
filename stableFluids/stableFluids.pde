import controlP5.*;

// global variables
final int xn = 120; //x軸のグリッドの数
final int yn = 80; //y軸のグリッドの数
final float h = 1.f/max(xn, yn); //微小距離Δx
float dt = 1.f/60; //微小時間Δt //シミュレーション上でのフレーム間の時間
int max_gsIterate = 50; //反復法の最大反復数
float srcRad = 4; //速度やインク等を書き込む範囲の半径
float srcVelAmp = 0.1; //書き込む速度の大きさ

// 配列のid
int curr_v = 0; //current velocity
int prev_v = 1; //previous velocity

int curr_i = 0; //current ink
int prev_i = 1; //previous ink

// 配列
float[][][] u = new float[2][xn][yn]; //速度のx方向の速度 [0][gn][gn]と[1][gn][gn]を使う
float[][][] v = new float[2][xn][yn]; //速度のy方向の速度 [0][gn][gn]と[1][gn][gn]を使う
float[][] div = new float[xn][yn]; //速度の発散 divergence
float[][] prs = new float[xn][yn]; //圧力 pressure
float[][][] ink = new float[2][xn][yn]; //インク

//コントトロール用フラグ
boolean isSimulate = true; //tureのときシミュレーションを行う
boolean isDisplayGrids = false; //tureのときグリッドを表示
boolean isDisplayVel = false; //tureのとき速度を表示

//GUI
ControlP5 cp5;
Accordion accordion;

void setup() {
  frameRate(60);
  size(600, 450); //xn:ynの比率と一緒にしたほうがいいです
  
  //GUIの初期設定
  initGUI();
  //配列の初期設定
  initArrays();
}

void draw() {
  background(0,255);
  
  //シミュレーションを行う
  if (isSimulate) updateSolver();
  
  //インクを表示する
  displayTiles(ink[curr_i]);
  //壁を描画する
  displayWall();
  
  //グリッドの表示
  if (isDisplayGrids) {
    stroke(127);
    displayGrids(); 
  }
  
  //速度の表示
  if (isDisplayVel) {
    stroke(255, 200, 0);
    displayVector(u[curr_v], v[curr_v]);
  }
  
  //FPSを表示する
  displayFPS();
}

void keyPressed() {
  if (key == ESC) exit();
  if (key == 'r') reset();
}

// GUI events // resetのボタンが押された時に実行
public void resetButton() {
  //シミュレーション結果をリセット
  reset();
  println("### resetButton(). a bang event. reset array");
}
