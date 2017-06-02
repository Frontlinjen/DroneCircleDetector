





void RingEstimation::Recieve(CircleResult* result){
  circleResults.add(result); //Not thread safe
}

void RingEstimation::Recieve(QRScanResult* result){
  scanResults.add(result);
}

void RingEstimation::Run(){
  running = true;
  while(running){
    
  }

}
