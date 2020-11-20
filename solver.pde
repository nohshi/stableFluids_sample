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
  addSourceInk();
  
  //圧力項
  projectVelocity();
  
  //移流項
  advectVelocity();
  advectInk();
  
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

// 外力項 // 速度を流し込む
void addSourceVelocity() {
  //マウスが押されていないと計算しない
  if (!mousePressed) return;
  
  //前フレームのマウスの値を使って、マウスの移動速度を算出
  PVector mouseVel = new PVector(mouseX - pmouseX, mouseY - pmouseY);
  mouseVel.mult(srcVelAmp);
  
  //マウス座標をグリッド上での座標(0.~xn or yn.)にする
  float mx = float(mouseX) * xn / width; //mouse x
  float my = float(mouseY) * yn / height; //mouse y
  
  //壁グリッド以外を全て参照
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {
      //マウスを中心に円状に広がる重み付けの数値pctを計算する
      /*
      PVector p = new PVector(i - mx, j - my);
      p.div(srcRad);
      float pct = max(1 - p.dot(p) ,0);
      */
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


// 外力項 // インクを流し込む
void addSourceInk() {
  //マウスが押されていないと計算しない
  if (!mousePressed) return;
  
  //マウス座標をグリッド上での座標(0 ~ xn-1 or yn-1)にする
  float mx = float(mouseX) * xn / width;
  float my = float(mouseY) * yn / height;
  
  //壁グリッド以外を全て参照
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {
      //マウスを中心に円状に広がる重み付けの数値pctを計算する
      /*
      PVector p = new PVector(i - mx, j - my);
      p.div(srcRad);
      float pct = max(1 - p.dot(p) ,0);
      */
      float pct = 1 - dist(i,j, mx, my) / srcRad;
      pct = max(0, pct);
      //マウス速度と重み付け関数を掛け合わせて速度に足す
      ink[curr_i][i][j] += pct * 255 * srcInkAmp;
      ink[curr_i][i][j] = min(ink[curr_i][i][j], 255);
    }
  }
}


//圧力項
void projectVelocity() {
  //速度の発散を事前計算
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {
      // gnを掛けるのはhで割るのと同義
      div[i][j] = - 0.5 * h * (u[curr_v][i+1][j] - u[curr_v][i-1][j] + v[curr_v][i][j+1] - v[curr_v][i][j-1]) / dt;
    }
  }
  
  //ガウス=ザイデル反復法を用いて圧力を求める
  float tolerance = 0.001; //許容誤差
  int iterate = 0;
  float err = 0; //誤差
  for (int n=0; n<max_gsIterate; n++) {
    err = 0; //誤差
    for (int i=1; i<xn-1; i++) {
      for (int j=1; j<yn-1; j++) {
        float prevPrs = prs[i][j];
        prs[i][j] = (prs[i+1][j] + prs[i-1][j] + prs[i][j+1] + prs[i][j-1] + div[i][j]) / 4.f;
        //err = max(err ,abs((prevPrs-prs[i][j])/prs[i][j]));
        err = max(err ,abs(prevPrs-prs[i][j]));
      }
    }
    //境界条件を強制する
    enforceWallPressure();
    iterate ++;
    //収束判定
    if (tolerance > err) break;
  }
  print("project::err : " + err + "\n");
  print("project::gs_iterate : " + iterate + "\n");
  
  //圧力の勾配を求めて速度を修正する
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {
      //x方向
      float gradPrsX = 0.5 * (prs[i+1][j] - prs[i-1][j]) / h;
      u[curr_v][i][j] -= dt * gradPrsX;
      //y方向
      float gradPrsY = 0.5 * (prs[i][j+1] - prs[i][j-1]) / h;
      v[curr_v][i][j] -= dt * gradPrsY;
    }
  }
}



//速度の移流項
void advectVelocity() {
  // currとprevのidの入れ替え. 交互に0 ⇄ 1, 1 ⇄ 0となる
  curr_v = 1 - curr_v;
  prev_v = 1 - prev_v;
  
  //壁グリッド以外を全て参照
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {
      
      //px,pyは移流点のxy座標。格子の中心なので+0.5する
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
      //
      int idx = floor(px);
      int idy = floor(py);
      //流体格子内に収まるよう強制
      idx = constrain(idx, 1, xn-2);
      idy = constrain(idy, 1, yn-2);
      //
      float v00, v10, v01, v11;
      float s = px - idx;
      float t = py - idy;
      
      v00 = u[prev_v][idx][idy];
      v10 = u[prev_v][idx+1][idy];
      v01 = u[prev_v][idx][idy+1];
      v11 = u[prev_v][idx+1][idy+1];
      float vx = bilinear(s,t, v00, v10, v01, v11);
      
      v00 = v[prev_v][idx][idy];
      v10 = v[prev_v][idx+1][idy];
      v01 = v[prev_v][idx][idy+1];
      v11 = v[prev_v][idx+1][idy+1];
      float vy = bilinear(s,t, v00, v10, v01, v11);
      
      u[curr_v][i][j] = vx;
      v[curr_v][i][j] = vy;
    }
  }
}

// インクの移流
void advectInk() {
  // currとprevのidの入れ替え. 交互に0 ⇄ 1, 1 ⇄ 0となる
  curr_i = 1 - curr_i;
  prev_i = 1 - prev_i;
  
  //壁グリッド以外を全て参照
  for (int i=1; i<xn-1; i++) {
    for (int j=1; j<yn-1; j++) {
      
      //px,pyは移流点のxy座標。格子の中心なので+0.5する
      float px = i;
      float py = j;
      //px,pyを0~1の空間に座標変換（hを掛けるのはgnで割るのと同じ。掛け算の方が速いのでこうする）
      px *= h;
      py *= h;
      //バックトレースで1フレーム前の位置を求める
      px -= u[prev_v][i][j] * dt;
      py -= v[prev_v][i][j] * dt;
      //px,pyを元の空間(0~gn)の空間に戻す
      px /= h;
      py /= h;
      //
      int idx = floor(px);
      int idy = floor(py);
      //流体格子内に収まるよう強制
      idx = constrain(idx, 1, xn-2);
      idy = constrain(idy, 1, yn-2);
      //
      float v00, v10, v01, v11;
      float s = px - idx;
      float t = py - idy;
      
      v00 = ink[prev_i][idx][idy];
      v10 = ink[prev_i][idx+1][idy];
      v01 = ink[prev_i][idx][idy+1];
      v11 = ink[prev_i][idx+1][idy+1];
      
      ink[curr_i][i][j] = bilinear(s,t, v00, v10, v01, v11);;
    }
  }
}

//壁のインク値を指定
void enforceWallInk() {
  //端のグリッドを参照
  for (int n=0; n<xn; n++) {
    ink[curr_i][n][0]  =  ink[curr_i][n][1];
    ink[curr_i][n][yn-1] =  ink[curr_i][n][yn-2];
  }
  for (int n=0; n<yn; n++) {
    ink[curr_i][0][n]  =  ink[curr_i][1][n];
    ink[curr_i][xn-1][n] =  ink[curr_i][xn-2][n];
  }
  //四隅は縦横で平均
  ink[curr_i][0][0]   = 0.5 * (ink[curr_i][1][0] + ink[curr_i][0][1]);
  ink[curr_i][xn-1][0]  = 0.5 * (ink[curr_i][xn-2][0] + ink[curr_i][xn-1][1]);
  ink[curr_i][0][yn-1]  = 0.5 * (ink[curr_i][1][yn-1] + ink[curr_i][0][yn-2]);
  ink[curr_i][xn-1][yn-1] = 0.5 * (ink[curr_i][xn-2][yn-1] + ink[curr_i][xn-1][yn-2]);
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
