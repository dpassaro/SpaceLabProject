void MMsensor_analysis(char* fileName, int seconds=60){

    // Silence warnings during TTree::ReadFile
    Int_t currentIgnoreLevel = gErrorIgnoreLevel;
    gErrorIgnoreLevel = kError;
    //read the file and access the TTree
    TTree *t = new TTree("t", "MM_data");
    // !!!!!! Verificare struttura dei dati dalla riga di disclaimer #---- dal file.log in esame !!!!!!!!!!!!!
    // per i file prima del 27/10/2021 non sono presente i voltaggi, per il file del 27/10/2021 sta come prima colonna
    t->ReadFile(fileName, "Timestamp/D:Pressure/D:Temp/D:Humidity/D");
    // Restore previous settings
    gErrorIgnoreLevel = currentIgnoreLevel;
    t->Print();

    //define the variables to be read in the TTree
    double Timestamp, Pressure, Temp, Humidity;
    int IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7;
    int AND0, AND1, AND2;

    t->SetBranchAddress("Timestamp", &Timestamp);
    t->SetBranchAddress("Pressure", &Pressure);
    t->SetBranchAddress("Temp", &Temp);
    t->SetBranchAddress("Humidity", &Humidity);
    //Get the number of entries in the TTree
    int n = t->GetEntries();
    double first_timestamp = 0.;
    double time, time_save;
    auto* gr_pres = new TGraphErrors();
    auto* gr_temp = new TGraphErrors();
    auto* gr_hum = new TGraphErrors();

    for(int ev=0; ev<=n*60/seconds; ++ev){//*10/seconds
        //read the current event
        t->GetEntry(ev*seconds/60);

        if(ev==0) first_timestamp = Timestamp;
        if(ev%1000==0)   cout<< "Event #"<< ev*seconds/10<< endl;

        time = (Timestamp-first_timestamp) / 3600.;
        if(time>=0){ time_save = time; }
        else {time = time_save +Timestamp/3600;}

        gr_temp->SetPoint(ev, time, Temp);
        gr_pres->SetPoint(ev, time, Pressure);
        gr_hum->SetPoint(ev, time, Humidity);
    }
    auto canvas = new TCanvas("canvas_MM","",1300,500);
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
