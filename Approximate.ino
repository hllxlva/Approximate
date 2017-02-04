//四捨五入の関数作らな...

int c[4][2] = {//センサーの位置
  { 300, 0},
  { 0, 300},
  {-300, 0},
  { 0,-300}
};
float data[4] = {0,0.1,0.2,0.1};//エンコーダの値

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
float Cr[4];//センサーまでの距離

void setup() {
  Serial.begin(250000);
}

int Mode(int num[8]){//最頻値(同じ値がない場合引数の精度を荒くする)
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
  }else{//同じ値がない
    for (int i = 0; i < 8; i++){//精度を荒くする
      num[i] = num[i]/2;
      num[i] = 2*num[i];
    }
  }
}

void Approx(int C[4][2],float Vd[4]){//センサーの位置，エンコーダーの値から自己位置を推定する．
  if(n == 1){//初期設定
    float Cp[4][2];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 2; j++) {
        Cp[i][j] = float(C[i][j])/100;//値が大きすぎてintやと足りない…
      }
    }
    for (int i = 0; i < 4; i++) {
      Cr[i] = 100*sqrt(Cp[i][0]*Cp[i][0]+Cp[i][1]*Cp[i][1]);//極座標系に
    }
    n = 0;
  }
  //float w[2];//ω
  for(int i = 0; i < 2; i++){//ω４つのセンサーから２通りωが出せる．
    now_v[2][i] = (Vd[i]+Vd[i+2])/(Cr[i]+Cr[i+2]);
  }
  //dif_v[2] = abs(now_v[2][0]-now_v[2][1]);
  //if(dif_v < 1){
    for (int i = 0; i < 2; i++){
      for (int j = 0; j < 2; j++){//速度の各成分は４通りの出し方がある
        now_v[0][2*i+j] = (-2*i+1)*(Cr[2*i+1]*now_v[2][j]-Vd[2*i+1]);
        now_v[1][2*i+j] = (-2*i+1)*(Vd[2*i]-Cr[2*i]*now_v[2][j]);
      }
    }
    for (int i = 0; i < 2; i++){//θを求める
      now_p[2][i] = now_p[2][i] + now_v[2][i];
      if(now_p[2][i] > 2*PI)now_p[2][i] - 2*PI;
    }
    dif_p[2] = abs(now_p[2][0]-now_p[2][1]);
    float now_p_ave[3];
    now_p_ave[2] = (now_p[2][0]+now_p[2][1])/2;
    for (int i = 0; i < 4; i++){//位置の各成分はここの４通りと
      now_p[0][i] = now_p[0][i]+(now_v[0][i]*cos(now_p[2][0])-now_v[1][i]*sin(now_p[2][0]));
      now_p[1][i] = now_p[1][i]+(now_v[0][i]*sin(now_p[2][0])+now_v[1][i]*cos(now_p[2][0]));
    }
    for (int i = 0; i < 4; i++){//ここの４通りで求めることが出来る．
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
      if (i == 2)now_p_int[i] = 180/PI*(now_p_ave[i])*10;//精度欲しいから10倍しとく
      else now_p_int[i] = now_p_ave[i];//0:X, 1:Y, 2:Ang をint型に直す．
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
  Serial.print(",");
  Serial.print(now_p_int[2]);
  Serial.print("|");
  Serial.println(dt);
}
