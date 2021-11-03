void correlation_analysis(char* fileName_datalogger, char* fileName_xlr8, int seconds = 1){
//******************************************************************************************
// Assicurarsi che i file siano sincronizzati !!!
//******************************************************************************************

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
    double Up_Time, UTC, Date, RMC_Valid, Sats_in_use, Latitude, Longitude, SoG,Sog,CoG,Altitude; // produce un errore, ma è l'unico modo per leggere il timestamp
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
    int k = dl->GetEntries();

    //define other convenient variables and counters
    double first_timestamp = 0.;
    int cont0 = 0;
    int cont1 = 0;
    int cont2 = 0;
    int cont6 = 0;
    int cont7 = 0;

    int const BINS_TEMP=27; int const  TEMP_LOW=18; int const TEMP_HIGH=24;
    int const BINS_PRES=27; int const PRES_LOW=1015; int const PRES_HIGH=1030;
    int const BINS_HUM=27; int const HUM_LOW=27; int const HUM_HIGH=36;
    int const BINS_CONT=35; int const CONT_LOW=0; int const CONT_HIGH=90;
    //Crates the Graph for the plot
    auto *gr0_temp = new TH2D("","IN0; Temperatura [#circC]; Conteggi", BINS_TEMP, TEMP_LOW,TEMP_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr1_temp = new TH2D("","Correlazione IN1/Temp; Temperatura [#circC]; Conteggi", BINS_TEMP, TEMP_LOW,TEMP_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr2_temp = new TH2D("","IN2; Temperatura [#circC]; Conteggi", BINS_TEMP, TEMP_LOW,TEMP_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr6_temp = new TH2D("","IN6; Temperatura [#circC]; Conteggi", BINS_TEMP, TEMP_LOW,TEMP_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr7_temp = new TH2D("","IN7; Temperatura [#circC]; Conteggi", BINS_TEMP, TEMP_LOW,TEMP_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr0_pres = new TH2D("","IN0; Pressione [mbar]; Conteggi", BINS_PRES, PRES_LOW,PRES_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr1_pres = new TH2D("","Correlazione IN1/Pres; Pressione [mbar]; Conteggi", BINS_PRES, PRES_LOW,PRES_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr2_pres = new TH2D("","IN2; Pressione [mbar]; Conteggi", BINS_PRES, PRES_LOW,PRES_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr6_pres = new TH2D("","IN6; Pressione [mbar]; Conteggi", BINS_PRES, PRES_LOW,PRES_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr7_pres = new TH2D("","IN7; Pressione [mbar]; Conteggi", BINS_PRES, PRES_LOW,PRES_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr0_hum = new TH2D("","IN0; Umidita' [%]; Conteggi", BINS_HUM, HUM_LOW, HUM_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr1_hum = new TH2D("","Correlazione IN1/Hum; Umidita' [%]; Conteggi", BINS_HUM, HUM_LOW, HUM_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr2_hum = new TH2D("","IN2; Umidita' [%]; Conteggi", BINS_HUM, HUM_LOW, HUM_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr6_hum = new TH2D("","IN3; Umidita' [%]; Conteggi", BINS_HUM, HUM_LOW, HUM_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );
    auto *gr7_hum = new TH2D("","IN4; Umidita' [%]; Conteggi", BINS_HUM, HUM_LOW, HUM_HIGH, BINS_CONT, CONT_LOW, CONT_HIGH );

    //loop on events
    int ev_dl = 0;
    int ev = 0; int ev_save = 0;
    bool flag=false;
    for(ev; ev<=n/seconds*1.; ++ev){// *10/seconds
        //read the current event
        t->GetEntry(ev*seconds);
        if(ev==0) first_timestamp = Timestamp;
        //-------- Sincronizza gli eventi ------------//
        for(ev_dl; ev_dl<k; ev_dl++){
          dl->GetEntry(ev_dl);
          //if(Timestamp_dl<first_timestamp && ev_dl>10) Timestamp_dl += first_timestamp;
          //if(Timestamp<first_timestamp) Timestamp += first_timestamp;

          if(std::abs(Timestamp_dl-Timestamp)<=1.5){
            break;
          }

          else if(Timestamp_dl>Timestamp && std::abs(Timestamp_dl-Timestamp)>1.5){
            ev_dl++;
            dl->GetEntry(ev_dl);
            //if(Timestamp_dl<first_timestamp) Timestamp_dl += first_timestamp;
            if(std::abs(Timestamp_dl-Timestamp)<=1.5) {
              break;}
            //debug
            //cout<<ev_dl<<endl;
            ev_save = ev*seconds;
            ev++;
            for(ev_save; ev_save<=n; ev_save++,ev++){
              //debug
              cout<<"ev_dl="<<Timestamp_dl<<"\t"<<"ev="<<Timestamp<<endl;//-first_timestamp
              t->GetEntry(ev_save);
              //if(Timestamp<first_timestamp) Timestamp += first_timestamp;

              if(std::abs(Timestamp_dl-Timestamp)<=1.5){
                flag = true;
                break;
              }
            }
            if(flag){
              break;
            }
            flag=false;
          }

          else if(Timestamp_dl<Timestamp && std::abs(Timestamp_dl-Timestamp)>1.2){
            continue;
          }

        }
        //-------------------------------------------//
            //
        if(ev%1000==0)   cout<< "XLR8: Timestamp         "<< Timestamp-first_timestamp<< "\t"<<ev<<endl;
        if(ev%1000==0)   cout<< "datalogger: Timestamp   "<< Timestamp_dl-first_timestamp<<"\t"<<ev_dl<<endl<<endl;

        //fill the Graph
        gr0_temp->Fill(ExtTemp, IN0-cont0);
        gr0_pres->Fill(Pressure, IN0-cont0);
        gr0_hum->Fill(Humidity, IN0-cont0);

        gr1_temp->Fill(ExtTemp, IN1-cont1);
        gr1_pres->Fill(Pressure, IN1-cont1);
        gr1_hum->Fill(Humidity, IN1-cont1);

        gr2_temp->Fill(ExtTemp, IN2-cont2);
        gr2_pres->Fill(Pressure, IN2-cont2);
        gr2_hum->Fill(Humidity, IN2-cont2);

        gr6_temp->Fill(ExtTemp, IN6-cont6);
        gr6_pres->Fill(Pressure, IN6-cont6);
        gr6_hum->Fill(Humidity, IN6-cont6);

        gr7_temp->Fill(ExtTemp, IN7-cont7);
        gr7_pres->Fill(Pressure, IN7-cont7);
        gr7_hum->Fill(Humidity, IN7-cont7);

        if(IN7<cont7) cout<<"Event number: "<<ev<<endl;
        //update counters
        cont0 = IN0;
        cont1 = IN1;
        cont2 = IN2;
        cont6 = IN6;
        cont7 = IN7;

    }

    //Crates the frame for the plot
    auto canvas = new TCanvas("canvas_corr_DL","",1300,500);
    canvas->Divide(3,1);
    canvas->cd(1);
    // cambiare il numero dei gra*_temp ecc per vedere gli altri contatori
    //Bellurie for the plot
    gr1_temp->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    //gPad->BuildLegend();

    canvas->cd(2);
    //Bellurie for the plot
    gr1_pres->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    //gPad->BuildLegend();

    canvas->cd(3);
    //Bellurie for the plot
    gr1_hum->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    //gPad->BuildLegend();

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
