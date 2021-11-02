void correlation_analysis(char* fileName_datalogger, char* fileName_xlr8, int seconds=10){

    /*for(int i=0; i<6; i++){
      if(fileName_datalogger[i+]!=fileName_xlr8[i]){
        cout<<"******************************************\n"<<endl;
        cout<<"Usare file relativi alla stessa presa dati\n"<<endl;
        cout<<"******************************************"<<endl;
        exit(EXIT_FAILURE);
      }
    }*/
    // Silence warnings during TTree::ReadFile
    Int_t currentIgnoreLevel = gErrorIgnoreLevel;
    gErrorIgnoreLevel = kError;
    //read the file and access the TTree
    TTree *dl = new TTree("dl", "datalogger");
    dl->ReadFile(fileName_datalogger, "Timestamp/D:Up_Time/C:UTC/C:Date/C:RMC_Valid/C:Sats_in_use/C:Latitude/C:Longitude/C:SoG/C:Sog/C:CoG/C:Altitude/C:BoardTemp/D:ExtTemp/D:Humidity/D:Pressure/D:Battery_Voltage/D:Logger_status/I");
    //dl->ReadFile(fileName_datalogger, "Timestamp_dl/D:Logger_status/I");
    dl->Print();

    TTree *t = new TTree("t", "XLR8");
    t->ReadFile(fileName_xlr8, "Timestamp/D:Reset_time/I:TRIGGER/I:IN0/I:IN1/I:IN2/I:IN3/I:IN4/I:IN5/I:IN6/I:IN7/I:AND0/I:AND1/I:AND2/I:V_THR1/D:V_THR2/D:V_BIAS/D");
    t->Print();
    // Restore previous settings
    gErrorIgnoreLevel = currentIgnoreLevel;

    //define the variables to be read in the TTree
    double Timestamp, V_bias, V_thr1, V_thr2;
    int IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7;
    int AND0, AND1, AND2;

    //Set the proper addresses
    t->SetBranchAddress("Timestamp", &Timestamp);
    t->SetBranchAddress("IN0", &IN0);
    t->SetBranchAddress("IN1", &IN1);
    t->SetBranchAddress("IN2", &IN2);
    t->SetBranchAddress("IN3", &IN3);
    t->SetBranchAddress("IN4", &IN4);
    t->SetBranchAddress("IN5", &IN5);
    t->SetBranchAddress("IN6", &IN6);
    t->SetBranchAddress("IN7", &IN7);


    double Timestamp_dl;
    char Up_Time, UTC, Date, RMC_Valid, Sats_in_use, Latitude, Longitude, SoG,Sog,CoG,Altitude;
    double BoardTemp,ExtTemp,Humidity,Pressure,Battery_Voltage;
    int Logger_status;

    //Set the proper address
    dl->SetBranchAddress("Timestamp", &Timestamp_dl);
    dl->SetBranchAddress("Up_Time", &Up_Time);
    dl->SetBranchAddress("UTC", &UTC);
    dl->SetBranchAddress("Date", &Date);
    dl->SetBranchAddress("RMC_Valid", &RMC_Valid);
    dl->SetBranchAddress("Sats_in_use", &Sats_in_use);
    dl->SetBranchAddress("Latitude", &Latitude);
    dl->SetBranchAddress("Longitude", &Longitude);
    dl->SetBranchAddress("SoG", &SoG);
    dl->SetBranchAddress("Sog", &Sog);
    dl->SetBranchAddress("CoG", &CoG);
    dl->SetBranchAddress("Altitude", &Altitude);
    dl->SetBranchAddress("BoardTemp", &BoardTemp);
    dl->SetBranchAddress("ExtTemp", &ExtTemp);
    dl->SetBranchAddress("Humidity", &Humidity);
    dl->SetBranchAddress("Pressure", &Pressure);
    dl->SetBranchAddress("Battery_Voltage", &Battery_Voltage);
    dl->SetBranchAddress("Logger_status", &Logger_status);
    //Timestamp[ms];Up-Time;UTC;Date;RMC Valid;Sats in use;Latitude;Longitude;Speed over ground[knots];Speed over ground[km/h];Course over ground;
    //Altitude NN [m];Board Temp[°C];External Temp[°C];Humidity[%];Pressure[hPa];Battery Voltage[V];Logger status
    //Get the number of entries in the TTree
    int n = t->GetEntries();

    //define other convenient variables and counters
    double first_timestamp = 0.;
    double last_timestamp = 0.;
    double aquisition_time = 0.;
    int cont0 = 0;
    int cont1 = 0;
    int cont2 = 0;
    int cont6 = 0;
    int cont7 = 0;

    //Crates the Graph for the plot
    auto *gr0_temp = new TGraphErrors();
    auto *gr1_temp = new TGraphErrors();
    auto *gr2_temp = new TGraphErrors();
    auto *gr6_temp = new TGraphErrors();
    auto *gr7_temp = new TGraphErrors();
    auto *gr0_pres = new TGraphErrors();
    auto *gr1_pres = new TGraphErrors();
    auto *gr2_pres = new TGraphErrors();
    auto *gr6_pres = new TGraphErrors();
    auto *gr7_pres = new TGraphErrors();
    auto *gr0_hum = new TGraphErrors();
    auto *gr1_hum = new TGraphErrors();
    auto *gr2_hum = new TGraphErrors();
    auto *gr6_hum = new TGraphErrors();
    auto *gr7_hum = new TGraphErrors();

    //loop on events
    for(int ev=0; ev<=n*10/seconds; ++ev){// *10/seconds
        //read the current event
        t->GetEntry(ev*seconds/10);
        dl->GetEntry(ev*seconds/2);
            //
        if(ev%1000==0)   cout<< "XLR8: Event "<< ev*seconds/10<< "\t IN0 " <<IN0<< endl;
        if(ev%1000==0)   cout<< "datalogger: Event "<< ev*seconds/2 <<"\t Temp "<< ExtTemp <<endl;

        //fill the Graph
        gr0_temp->SetPoint(ev, ExtTemp, IN0-cont0);
        gr0_pres->SetPoint(ev, Pressure, IN0-cont0);
        gr0_hum->SetPoint(ev, Humidity, IN0-cont0);

        gr1_temp->SetPoint(ev, ExtTemp, IN1-cont1);
        gr1_pres->SetPoint(ev, Pressure, IN1-cont1);
        gr1_hum->SetPoint(ev, Humidity, IN1-cont1);

        gr2_temp->SetPoint(ev, ExtTemp, IN2-cont2);
        gr2_pres->SetPoint(ev, Pressure, IN2-cont2);
        gr2_hum->SetPoint(ev, Humidity, IN2-cont2);

        gr6_temp->SetPoint(ev, ExtTemp, IN6-cont6);
        gr6_pres->SetPoint(ev, Pressure, IN6-cont6);
        gr6_hum->SetPoint(ev, Humidity, IN6-cont6);

        gr7_temp->SetPoint(ev, ExtTemp, IN7-cont7);
        gr7_pres->SetPoint(ev, Pressure, IN7-cont7);
        gr7_hum->SetPoint(ev, Humidity, IN7-cont7);


        if(IN7<cont7) cout<<"Event number: "<<ev*seconds/10<<endl;
        //update counters
        cont0 = IN0;
        cont1 = IN1;
        cont2 = IN2;
        cont6 = IN6;
        cont7 = IN7;


    }

    //Crates the frame for the plot
    auto canvas = new TCanvas("canvas_corr_DL","",1000,800);
    canvas->Divide(3,1);
    canvas->cd(1);
    //Bellurie for the plot
    gr0_temp->SetTitle("IN0"); gr0_temp->SetMarkerStyle(8); gr0_temp->SetMarkerColor(kRed);
    gr0_temp->SetLineColor(kRed); gr0_temp->SetMarkerSize(2);

    gr1_temp->SetTitle("IN1"); gr1_temp->SetMarkerStyle(8); gr1_temp->SetMarkerColor(kBlue);
    gr1_temp->SetLineColor(kBlue); gr1_temp->SetMarkerSize(2);

    gr2_temp->SetTitle("IN2"); gr2_temp->SetMarkerStyle(8); gr2_temp->SetMarkerColor(kBlack);
    gr2_temp->SetLineColor(kBlack); gr2_temp->SetMarkerSize(2);

    gr6_temp->SetTitle("IN6"); gr6_temp->SetMarkerStyle(8); gr6_temp->SetMarkerColor(kOrange);
    gr6_temp->SetLineColor(kOrange); gr6_temp->SetMarkerSize(2);

    gr7_temp->SetTitle("IN7"); gr7_temp->SetMarkerStyle(8); gr7_temp->SetMarkerColor(kGreen+2);
    gr7_temp->SetLineColor(kGreen+2); gr7_temp->SetMarkerSize(2);

    auto mg_temp = new TMultiGraph("mg_temp","mg_temp");
    //mg_temp->Add(gr0_temp, "PL");
    mg_temp->Add(gr1_temp, "P");
    //mg_temp->Add(gr2_temp, "PL");
    //mg_temp->Add(gr6_temp, "PL");
    //mg_temp->Add(gr7_temp, "PL");
    mg_temp->Draw("AP");

    const char* Line0 = "Correlazione rate/temp; Temperatura [#circC]; Counts/60s";
    mg_temp->SetTitle(Line0);//"Single rate; Time[s]; Counts/10s"
    mg_temp->SetMinimum(0);//mg->SetMaximum(80);
    gPad->BuildLegend();
    gPad->SetGrid();

    canvas->cd(2);
    //Bellurie for the plot
    gr0_pres->SetTitle("IN0"); gr0_pres->SetMarkerStyle(8); gr0_pres->SetMarkerColor(kRed);
    gr0_pres->SetLineColor(kRed); gr0_pres->SetMarkerSize(2);

    gr1_pres->SetTitle("IN1"); gr1_pres->SetMarkerStyle(8); gr1_pres->SetMarkerColor(kBlue);
    gr1_pres->SetLineColor(kBlue); gr1_pres->SetMarkerSize(2);

    gr2_pres->SetTitle("IN2"); gr2_pres->SetMarkerStyle(8); gr2_pres->SetMarkerColor(kBlack);
    gr2_pres->SetLineColor(kBlack); gr2_pres->SetMarkerSize(2);

    gr6_pres->SetTitle("IN6"); gr6_pres->SetMarkerStyle(8); gr6_pres->SetMarkerColor(kOrange);
    gr6_pres->SetLineColor(kOrange); gr6_pres->SetMarkerSize(2);

    gr7_pres->SetTitle("IN7"); gr7_pres->SetMarkerStyle(8); gr7_pres->SetMarkerColor(kGreen+2);
    gr7_pres->SetLineColor(kGreen+2); gr7_pres->SetMarkerSize(2);

    auto mg_pres = new TMultiGraph("mg_pres","mg_pres");
    //mg_pres->Add(gr0_pres, "PL");
    mg_pres->Add(gr1_pres, "P");
    //mg_pres->Add(gr2_pres, "PL");
    //mg_pres->Add(gr6_pres, "PL");
    //mg_pres->Add(gr7_pres, "PL");
    mg_pres->Draw("AP");

    const char* Line1 = "Correlazione rate/pres; Pressione [mbar]; Counts/60s";
    mg_pres->SetTitle(Line1);//"Single rate; Time[s]; Counts/10s"
    mg_pres->SetMinimum(0);//mg->SetMaximum(80);
    gPad->BuildLegend();
    gPad->SetGrid();

    canvas->cd(3);
    //Bellurie for the plot
    gr0_hum->SetTitle("IN0"); gr0_hum->SetMarkerStyle(8); gr0_hum->SetMarkerColor(kRed);
    gr0_hum->SetLineColor(kRed); gr0_hum->SetMarkerSize(2);

    gr1_hum->SetTitle("IN1"); gr1_hum->SetMarkerStyle(8); gr1_hum->SetMarkerColor(kBlue);
    gr1_hum->SetLineColor(kBlue); gr1_hum->SetMarkerSize(2);

    gr2_hum->SetTitle("IN2"); gr2_hum->SetMarkerStyle(8); gr2_hum->SetMarkerColor(kBlack);
    gr2_hum->SetLineColor(kBlack); gr2_hum->SetMarkerSize(2);

    gr6_hum->SetTitle("IN6"); gr6_hum->SetMarkerStyle(8); gr6_hum->SetMarkerColor(kOrange);
    gr6_hum->SetLineColor(kOrange); gr6_hum->SetMarkerSize(2);

    gr7_hum->SetTitle("IN7"); gr7_hum->SetMarkerStyle(8); gr7_hum->SetMarkerColor(kGreen+2);
    gr7_hum->SetLineColor(kGreen+2); gr7_hum->SetMarkerSize(2);

    auto mg_hum = new TMultiGraph("mg_hum","mg_hum");
    //mg_hum->Add(gr0_hum, "PL");
    mg_hum->Add(gr1_hum, "P");
    //mg_hum->Add(gr2_hum, "PL");
    //mg_hum->Add(gr6_hum, "PL");
    //mg_hum->Add(gr7_hum, "PL");
    mg_hum->Draw("AP");

    const char* Line2 = "Correlazione rate/hum; Umidita' [%]; Counts/60s";
    mg_hum->SetTitle(Line2);//"Single rate; Time[s]; Counts/10s"
    mg_hum->SetMinimum(0);//mg->SetMaximum(80);
    gPad->BuildLegend();
    gPad->SetGrid();

}

void time_analysis(char* fileName, int seconds = 2){
  Int_t currentIgnoreLevel = gErrorIgnoreLevel;
  gErrorIgnoreLevel = kError;
  //read the file and access the TTree
  TTree *dl = new TTree("dl", "datalogger");
  dl->ReadFile(fileName, "Timestamp/D:Up_Time/C:UTC/C:Date/C:RMC_Valid/C:Sats_in_use/C:Latitude/C:Longitude/C:SoG/C:Sog/C:CoG/C:Altitude/C:BoardTemp/D:ExtTemp/D:Humidity/D:Pressure/D:Battery_Voltage/D:Logger_status/I");//
  //dl->ReadFile(fileName_datalogger, "Timestamp_dl/D:Logger_status/I");
  dl->Print();
  // Restore previous settings
  gErrorIgnoreLevel = currentIgnoreLevel;

  //define the variables to be read in the TTree
  double Timestamp;
  double Up_Time, UTC, Date, RMC_Valid, Sats_in_use, Latitude, Longitude, SoG,Sog,CoG,Altitude; // produce un errore, ma è l'unico modo per leggere bene il timestamp
  double BoardTemp,ExtTemp,Humidity,Pressure,Battery_Voltage;
  int Logger_status;

  //Set the proper address
  dl->SetBranchAddress("Timestamp", &Timestamp);
  dl->SetBranchAddress("Up_Time", &Up_Time);
  dl->SetBranchAddress("UTC", &UTC);
  dl->SetBranchAddress("Date", &Date);
  dl->SetBranchAddress("RMC_Valid", &RMC_Valid);
  dl->SetBranchAddress("Sats_in_use", &Sats_in_use);
  dl->SetBranchAddress("Latitude", &Latitude);
  dl->SetBranchAddress("Longitude", &Longitude);
  dl->SetBranchAddress("SoG", &SoG);
  dl->SetBranchAddress("Sog", &Sog);
  dl->SetBranchAddress("CoG", &CoG);
  dl->SetBranchAddress("Altitude", &Altitude);
  dl->SetBranchAddress("BoardTemp", &BoardTemp);
  dl->SetBranchAddress("ExtTemp", &ExtTemp);
  dl->SetBranchAddress("Humidity", &Humidity);
  dl->SetBranchAddress("Pressure", &Pressure);
  dl->SetBranchAddress("Battery_Voltage", &Battery_Voltage);
  dl->SetBranchAddress("Logger_status", &Logger_status);
  //Get the number of entries in the TTree
  int n = dl->GetEntries();
  auto *gr_temp = new TGraphErrors();
  auto *gr_pres = new TGraphErrors();
  auto *gr_hum = new TGraphErrors();
  double time, time_save;
  double first_timestamp = 0.;
  double last_timestamp = 0.;
  double aquisition_time = 0.;
  //loop on events
  for(int ev=0; ev<=n*2/seconds; ++ev){// *10/seconds
      //read the current event
      dl->GetEntry(ev*seconds/2);

      if(ev==0) first_timestamp = Timestamp;
          //
      if(ev%1000==0)   cout<< "datalogger: Event "<< ev*seconds/2 <<endl;

      time = (Timestamp-first_timestamp) / 3600.;
      if(time>=0){ time_save = time; }
      else {time = time_save +Timestamp/3600;}
      //fill the Graph
      gr_temp->SetPoint(ev, time, ExtTemp);
      gr_pres->SetPoint(ev, time, Pressure);
      gr_hum->SetPoint(ev, time, Humidity);
  }
  auto canvas = new TCanvas("canvas_time_DL","",1300,500);
  canvas->Divide(3,1);
  canvas->cd(1);
  //Bellurie for the plot
  gr_temp->SetTitle("Andamento Temperatura; Tempo [h]; Temperatura [#circC]"); gr_temp->SetMarkerStyle(8);
  gr_temp->SetMarkerColor(kRed); gr_temp->SetLineColor(kRed); gr_temp->SetMarkerSize(0.5);
  gr_temp->Draw("APL");
  gPad->SetGrid();

  canvas->cd(2);
  //Bellurie for the plot
  gr_pres->SetTitle("Andamento Pressione; Tempo [h]; Pressione [mbar]"); gr_pres->SetMarkerStyle(8);
  gr_pres->SetMarkerColor(kBlue); gr_pres->SetLineColor(kBlue); gr_pres->SetMarkerSize(0.5);
  gr_pres->Draw("APL");
  gPad->SetGrid();

  canvas->cd(3);
  //Bellurie for the plot
  gr_hum->SetTitle("Andamento Umidita'; Tempo [h]; Umidita' [%]"); gr_hum->SetMarkerStyle(8);
  gr_hum->SetMarkerColor(kGreen); gr_hum->SetLineColor(kGreen); gr_hum->SetMarkerSize(0.5);
  gr_hum->Draw("APL");
  gPad->SetGrid();
}
