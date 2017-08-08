
import processing.serial.*;

PrintWriter output;
 
int NUM = 1; //センサーの数
Serial myPort; // シリアルポート

final int WAITCOUNT = 20;
int[] temp = new int[WAITCOUNT];
int count = 0;

final int THRESHCOUNT = 50;
int thresh_count = 0;
int max = 0, min = 1000, thresh = 0, thresh_flag = 0;

static int detect_flag = 0;
int breath = 0;
static int compare_before = 0, compare_current = 0;

float bpmNow=0, bpmBefore=0;
float lastBreathTime=0, currentBreathTime=0;
 
int[] sensors = new int[NUM]; //センサーの値を格納する配列
int cnt; //カウンター
 
// グラフの線の色を格納
color[] col = new color[6];
float last_tx = 0;  
float last_ty = 0;

void setup() {
  //画面設定
  size(1000, 800);
  frameRate(60);

  myPort = new Serial( this, "COM6", 115200 );
  myPort.bufferUntil('\n');
  initGraph();
  
  String filename = nf(year(),4) + nf(month(),2) + nf(day(),2) + nf(hour(),2) + nf(minute(),2) ;
  output = createWriter( filename + ".csv");
  
  for(int i = 0; i < WAITCOUNT; i++){
   temp[i] = 0; 
  }
   
}
 
void draw() {
  
  if(sensors[0] > 0){
  
  //CALCULATE AVERAGE
  if(count <  WAITCOUNT){//COLLECT DATA FOR AVERAGE
    temp[count] = sensors[0];
    count++;
  }
  else{//GET AVERAGE & THRESH & PLOT TO GRAPH
   int average = 0;
   for(int i = 0; i < WAITCOUNT; i++){
    average += temp[i];
    temp[i] = 0;
   }
   average = average / WAITCOUNT;
   count = 0;
   //println("AVERAGE: " + average);
   float timer2 = millis();
   output.println(timer2 / 1000+",AVERAGE: " + average);
  
   //CALCULATE THRESHOLD
   if(thresh_count < THRESHCOUNT){
     if(average > max) max = average;
     else if(average < min) min = average;
     thresh_count ++;
     //println("MAX: " + max + " MIN: " + min);
    }
   else{
    if(detect_flag == 0) detect_flag = 1;
    thresh = (max+min) / 2;
    max = 0;
    min = 1000;
    thresh_count = 0;
   }
   //println("COUNT: " + thresh_count + " THRESH: " + thresh);

  // DRAW GRAPH
  for (int i = 0; i < NUM; i++) {
    
    stroke(col[i]);
    float tx = map(cnt, 0, width, 0, width);
    //float ty = map(sensors[i]*23, 0, 1023, height, 0);
    float ty = map(average*20, 0, 1023, height, 0);
    line(tx, ty, last_tx, last_ty);
    
    stroke(col[4]);
    float threshy = map(thresh*20, 0, 1023, height, 0);
    line(tx, threshy, last_tx, threshy);
    
    last_tx = tx;
    last_ty = ty;
    //println(tx+":"+ty);
    if(last_tx >= width) last_tx = 0;
  }
  if (cnt > width) {
    initGraph();
  }
  cnt+=2;
  
  //DETECT BREATH
  if(detect_flag == 1){
    compare_before = compare_current;
    compare_current = average;
    println("comparing");
    if(compare_before > thresh && compare_current <= thresh){
      breath++;
      output.println(timer2 / 1000+",BREATH_DETECTED");
      lastBreathTime = currentBreathTime;
      currentBreathTime = timer2;
      bpmBefore = bpmNow;
      bpmNow = 60 / ((currentBreathTime - lastBreathTime) / 1000);
      bpmNow = (bpmBefore + bpmNow) / 2;
      
      //draw rectangle to cover past number
      stroke(255);
      fill(47);
      rect(800, 0, 200, 200);
      //draw breath num
      fill(255);
      textSize(40);
      text(breath, 850, 50);
      text(bpmNow, 850, 150);
    }
  }
  
  }//close else
  }//close if(sensors[0] > 0)
}
 
//グラフの初期化
void initGraph() {
  background(47);
  strokeWeight(2);
  //noStroke();
  cnt = 0;
  // グラフ描画の線の色を定義
  col[0] = color(255, 127, 31);
  col[1] = color(31, 255, 127);
  col[2] = color(127, 31, 255);
  col[3] = color(31, 127, 255);
  col[4] = color(127, 255, 31);
  col[5] = color(127);
}
 
void serialEvent(Serial myPort) {
  try{
    String myString = myPort.readStringUntil('\n');
    myString = trim(myString);
    sensors = int(split(myString, ','));
    float timer = millis();
    output.println(timer / 1000+","+sensors[0]);
    println("SENSORS[0]: " + sensors[0]);
    //myPort.write("A");
    }
  catch(RuntimeException e) {
    e.printStackTrace();
  }
}

void keyPressed(){
  if( key == 'q' ){
    output.flush(); 
    output.close(); 
    exit();               
  }
}
