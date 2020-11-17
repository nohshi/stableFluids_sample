// initialize arrays
void initArrays() {
  clearArray(u[curr_v]);
  clearArray(u[prev_v]);
  clearArray(v[curr_v]);
  clearArray(v[prev_v]);
  clearArray(prs);
  clearArray(div);
  clearArray(ink[curr_i]);
  clearArray(ink[prev_i]);
}

void reset() {
  initArrays();
}

// シミュレーションを行う
void updateSolver() {
  
  //外力項 //速度とインクを流し込む
  addSourceVelocity();
  //addSourceInk();
  
  //圧力項
  //projectVelocity();
  
  //移流項
  //advectVelocity();
  //advectInk();
  
}


// 外力項 // 速度を流し込む
void addSourceVelocity() {
  //マウスが押されていないと計算しない
  if (!mousePressed) return;
  
  //前フレームのマウスの値を使って、マウスの移動速度を算出
  PVector mouseVel = new PVector(mouseX - pmouseX, mouseY - pmouseY);
  //srcVelAmpは力を加える範囲を調整するパラメーター。GUIで変更できます
  mouseVel.mult(srcVelAmp);
  
  //壁グリッド以外を全て参照
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {
      //マウス座標をグリッド上での座標(0.~xn or yn.)にする
      float mx = float(mouseX) * xn / width; //mouse x
      float my = float(mouseY) * yn / height; //mouse y
      //マウスを中心に円状に広がる重み付けの数値pctを計算する
      //srcRadは力を加える範囲を調整するパラメーター。GUIで変更できます
      float pct = 1 - dist(i,j, mx, my) / srcRad;
      pct = max(0, pct);
      //マウス速度と重み付け関数を掛け合わせて速度に足す
      PVector vel = PVector.mult(mouseVel, pct);
      vel.x += u[curr_v][i][j];
      vel.y += v[curr_v][i][j];
      vel.limit(5); //速さが大きくなりすぎないように制限する
      u[curr_v][i][j] = vel.x;
      v[curr_v][i][j] = vel.y;
    }
  }
}

//圧力項
void projectVelocity() {
  // 反復計算を行う前に圧力以外の値を事前計算する
  //--------------------------------------------------------
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {
      /*
          div[][]は連立方程式の右辺、つまり反復計算において変化の無い値を事前計算して保存しておきます。
          div[i][j]の値を計算してください
          div[i][j] = 
      */
    }
  }

  // ガウス=ザイデル反復法を用いて圧力を求める
  //--------------------------------------------------------
  float tolerance = 0.001; //許容誤差
  int iterate = 0; //反復回数のカウント
  float err = 0; //誤差
  for (int n=0; n<max_gsIterate; n++) {
    err = 0;
    for (int i=1; i<xn-1; i++) {
      for (int j=1; j<yn-1; j++) {
        float prevPrs = prs[i][j];
        /*
            ここに反復式を書いて、新しいprs[i][j]の値を求めてください
            prs[i][j] = 
        */  
        err = max(err ,abs(prevPrs-prs[i][j]));
      }
    }
    //境界条件を強制する
    enforceWallPressure();
    iterate ++;
    //収束判定
    if (tolerance > err) break;
  }
  //エラー値の出力
  print("project::err : " + err + "\n");
  //反復回数の出力
  print("project::gs_iterate : " + iterate + "\n");

  // 圧力の勾配∇pを求めて速度を修正する
  //--------------------------------------------------------
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {
      /*
          x軸方向, y軸方向それぞれにおける圧力の勾配∇pを求めてください
          //x軸方向
          float gradPrsX =
          //y軸方向
          float gradPrsY =
      */
      //最終的な速度を求めて加えます
      //u[curr_v][i][j] += - dt * gradPrsX; //x軸方向
      //v[curr_v][i][j] += - dt * gradPrsY; //y軸方向
    }
  }
}


//壁の速度を強制することで、境界グリッドの勾配を調整する //圧力項で使用
void enforceWallPressure() {
  //端のグリッドを参照
  for (int n=0; n<xn; n++) {
    prs[n][0]  =  prs[n][1];
    prs[n][yn-1] =  prs[n][yn-2];
  }
  for (int n=0; n<yn; n++) {
    prs[0][n]  =  prs[1][n];
    prs[xn-1][n] =  prs[xn-2][n];
  }
  //四隅は縦横で平均
  prs[0][0]   = 0.5 * (prs[1][0] + prs[0][1]);
  prs[xn-1][0]  = 0.5 * (prs[xn-2][0] + prs[xn-1][1]);
  prs[0][yn-1]  = 0.5 * (prs[1][yn-1] + prs[0][yn-2]);
  prs[xn-1][yn-1] = 0.5 * (prs[xn-2][yn-1] + prs[xn-1][yn-2]);
}


//速度の移流項
void advectVelocity() {
  // currとprevのidの入れ替え. 
  int tmp = curr_v;
  curr_v = prev_v;
  prev_v = tmp;

  //壁グリッド以外を全て参照
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {

      //px,pyは移流点のxy座標。
      float px = i;
      float py = j;
      //px,pyをほぼ0~1の空間に座標変換
      px *= h;
      py *= h;
      //バックトレースで1フレーム前の位置を求める
      px -= u[prev_v][i][j] * dt;
      py -= v[prev_v][i][j] * dt;
      //px,pyを元の空間(0~gn)の空間に戻す
      px /= h;
      py /= h;
      //近傍の4グリッドのインデックスを求める
      int idx = floor(px);
      int idy = floor(py);
      //流体格子内に収まるよう強制
      idx = constrain(idx, 1, xn-2);
      idy = constrain(idy, 1, yn-2);

      //近傍4グリッドの速度をu,vそれぞれバイリニア補間する
      float v00, v10, v01, v11;
      float s = px - idx;
      float t = py - idy;

      v00 = u[prev_v][idx][idy];
      v10 = u[prev_v][idx+1][idy];
      v01 = u[prev_v][idx][idy+1];
      v11 = u[prev_v][idx+1][idy+1];
      //バイリニア補間を行う関数 s,tは0~1、v00, v10, v01, v11は4隅の値
      float vx = bilinear(s,t, v00, v10, v01, v11);

      v00 = v[prev_v][idx][idy];
      v10 = v[prev_v][idx+1][idy];
      v01 = v[prev_v][idx][idy+1];
      v11 = v[prev_v][idx+1][idy+1];
      float vy = bilinear(s,t, v00, v10, v01, v11);

      //更新後の値をcurr_vの配列に格納
      u[curr_v][i][j] = vx;
      v[curr_v][i][j] = vy;
    }
  }
}


//壁の速度を強制することで、境界グリッドの勾配を調整する // 粘性項で使用
void enforceWallVelocityX() {
  //端のグリッドを参照
  for (int n=0; n<xn; n++) { //縦
    u[curr_v][n][0]    =   u[curr_v][n][1];
    u[curr_v][n][yn-1] =   u[curr_v][n][yn-2];
  }
  for (int n=0; n<yn; n++) { //横
    u[curr_v][0][n]    = - u[curr_v][1][n];
    u[curr_v][xn-1][n] = - u[curr_v][xn-2][n];
  }
  //四隅は縦横で平均
  u[curr_v][0][0]   = 0.5 * (u[curr_v][1][0] + u[curr_v][0][1]);
  u[curr_v][xn-1][0]  = 0.5 * (u[curr_v][xn-2][0] + u[curr_v][xn-1][1]);
  u[curr_v][0][yn-1]  = 0.5 * (u[curr_v][1][yn-1] + u[curr_v][0][yn-2]);
  u[curr_v][xn-1][yn-1] = 0.5 * (u[curr_v][xn-2][yn-1] + u[curr_v][xn-1][yn-2]);
}

//壁の速度を強制することで、境界グリッドの勾配を調整する // 粘性項で使用
void enforceWallVelocityY() {
  //端のグリッドを参照
  for (int n=0; n<xn; n++) { //縦
    v[curr_v][n][0]    = - v[curr_v][n][1];
    v[curr_v][n][yn-1] = - v[curr_v][n][yn-2];
  }
  for (int n=0; n<yn; n++) { //横
    v[curr_v][0][n]    =   v[curr_v][1][n];
    v[curr_v][xn-1][n] =   v[curr_v][xn-2][n];
  }
  //四隅は縦横で平均
  v[curr_v][0][0]   = 0.5 * (v[curr_v][1][0] + v[curr_v][0][1]);
  v[curr_v][xn-1][0]  = 0.5 * (v[curr_v][xn-2][0] + v[curr_v][xn-1][1]);
  v[curr_v][0][yn-1]  = 0.5 * (v[curr_v][1][yn-1] + v[curr_v][0][yn-2]);
  v[curr_v][xn-1][yn-1] = 0.5 * (v[curr_v][xn-2][yn-1] + v[curr_v][xn-1][yn-2]);
}
