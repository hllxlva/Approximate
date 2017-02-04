int c[4][2] = {
  { 300, 0},
  { 0, 300},
  {-300, 0},
  { 0,-300}
};
float data[4] = {0,0.1,0.2,0.1};
unsigned long time;
unsigned long t0;
float dt;


//-------------------------------
float now_p[3][8];//今の位置 0:X, 1:Y, 2:Ang
int now_p_int[3];//今の位置の平均 0:X, 1:Y, 2:Ang
float now_v[3][4];//今の速度 0:X, 1:Y, 2:Ang
float dif_v[3];//速度の違い 0:X, 1:Y, 2:Ang
float dif_p[3];//位置の違い 0:X, 1:Y, 2:Ang
int n = 1;//初期設定フラグ
float Cr[5];//センサーまでの距離　0 = 4

void setup() {
  Serial.begin(250000);
}

int Mode(int num[8]){
  int mode = 0;
  int cnt = 0;
  for(int i = 0; i < 8; i++){
    int temp_cnt = 1;
    for(int j = i + 1; j < 8; j++){
      if( num[i] == num[j]){
        temp_cnt++;
      }
    }
    if( temp_cnt > cnt){
      cnt = temp_cnt;
      mode = num[i]; 
    }
  }
  if (cnt > 1){
    return mode;
  }else{
    for (int i = 0; i < 8; i++){
      num[i] = num[i]/2;
      num[i] = 2*num[i];
    }
  }
}

void Approx(int C[5][2],float Vd[5]){
  if(n == 1){//初期設定
    float Cp[4][2];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 2; j++) {
        Cp[i][j] = float(C[i][j])/100;//値が大きすぎてintやと足りない…
      }
    }
    for (int i = 0; i < 4; i++) {
      Cr[i] = 100*sqrt(float(Cp[i][0]*Cp[i][0]+Cp[i][1]*Cp[i][1]));//極座標系に
    }
    Cr[4] = Cr[0];
    n = 0;
  }
  Vd[4] = Vd[0];
  float w[2];//ω
  for(int i = 0; i < 2; i++){
    w[i] = (Vd[i]+Vd[i+2])/(Cr[i]+Cr[i+2]);
  }
  for(int i = 0; i < 2; i++){
    now_v[2][i] = w[i];
  }
  //dif_v[2] = abs(w[0]-w[1]);
  float dx[4];
  float dy[4];
  //if(dif_v < 1){
    for (int i = 0; i < 2; i++){
      for (int j = 0; j < 2; j++){
        dx[2*i+j] = (-2*i+1)*(Cr[2*i+1]*w[j]-Vd[2*i+1]);
        dy[2*i+j] = (-2*i+1)*(Vd[2*i]-Cr[2*i]*w[j]);
      }
    }
    for (int i = 0; i < 4; i++){
      now_v[0][i] = dx[i];
      now_v[1][i] = dy[i];
    }
    for (int i = 0; i < 2; i++){
      now_p[2][i] = now_p[2][i] + now_v[2][i];
    }
    dif_p[2] = abs(now_p[2][0]-now_p[2][1]);
    float now_p_ave[3];
    now_p_ave[2] = (now_p[2][0]+now_p[2][1])/2;
    for (int i = 0; i < 4; i++){
      now_p[0][i] = now_p[0][i]+(now_v[0][i]*cos(now_p[2][0])-now_v[1][i]*sin(now_p[2][0]));
      now_p[1][i] = now_p[1][i]+(now_v[0][i]*sin(now_p[2][0])+now_v[1][i]*cos(now_p[2][0]));
    }
    for (int i = 0; i < 4; i++){
      now_p[0][i+4] = now_p[0][i+4]+(now_v[0][i]*cos(now_p[2][1])-now_v[1][i]*sin(now_p[2][1]));
      now_p[1][i+4] = now_p[1][i+4]+(now_v[0][i]*sin(now_p[2][1])+now_v[1][i]*cos(now_p[2][1]));
    }
    for (int i = 0; i < 2; i++){
      now_p_ave[i] = 0;
      for (int j = 0; j < 8; j++){
        now_p_ave[i] = (j*now_p_ave[i]+now_p[i][j])/(j+1);//重心を求める
      }
    }
    for (int i = 0; i < 3; i++){
      now_p_int[i] = now_p_ave[i];
    }
  //}
}

void loop() {
  time = micros();
  dt = float(time - t0)/1000;
  t0 = time;
  Approx(c, data);
  /*for (int i = 0; i < 2; i++){
    Serial.print(w[i]);
    Serial.print("|");  
  }*/
  Serial.print(now_p_int[0]);
  Serial.print(",");
  Serial.print(now_p_int[1]);
  Serial.print("|");
  Serial.println(dt);
}
