

// displayにおけるpixelsのidを得る
int dsp_id(int id_x, int id_y) {
  return min(id_x + id_y * width, width*height-1);
}

//バイリニア補間 //x=0~1, y=0~1 //v00~v11は四隅の値
float bilinear(float x, float y, float v00, float v10, float v01, float v11) {
    //x = constrain(x, 0, 1);
    //y = constrain(y, 0, 1);
    float xa = 1 - x;
    float ya = 1 - y;
    return v00*xa*ya + v01*xa*y + v10*x*ya + v11*x*y;
}

// 配列の中身を全て0にする
void clearArray(float[][] array) {
  for (int i=0; i<array.length ; i++) {
    for (int j=0; j<array[i].length ; j++) {
      array[i][j] = 0;
    }
  }
}

//グリッドの表示
void displayGrids() {
  for (float i=1; i<xn; i++) {
    float px = i / xn * width;
    line(px, 0, px, height);
  }
  for (float i=1; i<yn; i++) {
    float py = i / yn * height;
    line(0, py, width, py);
  }
}

// グリッドの中身を表示
void displayTiles(float[][] array) {
  loadPixels();
  for (int i=0; i<width; i++) {
    for (int j=0; j<height; j++) {
      int x = i * xn / width;
      int y = j * yn / height;
      pixels[dsp_id(i,j)] = color(array[x][y]*255);
    }
  }
  updatePixels();
}

// 壁に色をつけて表示
void displayWall() {
  loadPixels();
  for (int i=0; i<width; i++) {
    for (int j=0; j<height; j++) {
      int x = i * xn / width;
      int y = j * yn / height;
      if (x == 0 || y == 0 || x == xn-1 || y == yn-1 ) {
        pixels[dsp_id(i,j)] = color(60,60,150);
      }
    }
  }
  updatePixels();
}


//ベクトルを表示
void displayVector(float[][] _u, float[][] _v) {
  float l = width * h * 0.5;
  for (int i=0; i<xn; i++) {
    for (int j=0; j<yn; j++) {
      float px = (i+0.5) * width / xn;
      float py = (j+0.5) * height / yn;
      PVector v = new PVector(_u[i][j], _v[i][j]);
      v.mult(l);
      v.mult(4.1);
      float dstx = px + v.x;
      float dsty = py + v.y;
      line(px,py, dstx,dsty);
    }
  }
}


// fpsを表示
void displayFPS() {
  fill(0);
  rect(5, 5, 105, 20);
  fill(255);
  text("fps : " + frameRate, 10, 20);
}


// GUIの初期化、設定
// controlP5のgroup,toggle,ban,sliderなどのサンプルを参照してください
void initGUI() {
  cp5 = new ControlP5(this);
  
  // group number 1, contains 2 bangs
  Group g1 = cp5.addGroup("GUI")
                .setBackgroundColor(color(0, 150))
                .setBackgroundHeight(200)
                ;
                
     
  cp5.addSlider("srcVelAmp")
     .setPosition(0, 10)
     .setSize(130,10)
     .setRange(0, 0.4)
     .setLabel("srcVelAmp")
     .moveTo(g1);
 
 cp5.addSlider("srcRad")
     .setPosition(0, 30)
     .setSize(130,10)
     .setRange(0.01, 10)
     .setLabel("srcRad")
     .moveTo(g1);
     
 cp5.addSlider("max_gsIterate")
     .setPosition(0, 50)
     .setSize(130,10)
     .setRange(1, 20000)
     .setLabel("max_gsIterate")
     .moveTo(g1);
     
 
     // change the trigger event, by default it is PRESSED.
  cp5.addBang("resetButton")
     .setPosition(10, 100)
     .setSize(30, 30)
     .setTriggerEvent(Bang.PRESS)
     .setLabel("reset")
     .moveTo(g1);
     
     // change the trigger event, by default it is PRESSED.
  cp5.addToggle("isSimulate")
     .setPosition(80, 110)
     .setSize(50,20)
     .setMode(ControlP5.SWITCH)
     .setLabel("Simulate")
     .moveTo(g1);
     
  // create a toggle and change the default look to a (on/off) switch look
  cp5.addToggle("isDisplayGrids")
     .setPosition(10,150)
     .setSize(50,20)
     .setMode(ControlP5.SWITCH)
     .setLabel("Display Grids")
     .moveTo(g1);
     
  cp5.addToggle("isDisplayVel")
     .setPosition(80,150)
     .setSize(50,20)
     .setMode(ControlP5.SWITCH)
     .setLabel("Display Velocity")
     .moveTo(g1);
     
  // create a new accordion
  // add g1, g2, and g3 to the accordion.
  accordion = cp5.addAccordion("acc")
                 .setPosition(5,40)
                 .setWidth(200)
                 .addItem(g1);
                 
  accordion.open(0);
  
  // use Accordion.MULTI to allow multiple group 
  // to be open at a time.
  accordion.setCollapseMode(Accordion.MULTI);
}
