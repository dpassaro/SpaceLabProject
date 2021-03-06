void analysis(char* fileName, int seconds=10){

    // Silence warnings during TTree::ReadFile
    Int_t currentIgnoreLevel = gErrorIgnoreLevel;
    gErrorIgnoreLevel = kError;
    //read the file and access the TTree
    TTree *t = new TTree("t", "XLR8_data");
    // !!!!!! Verificare struttura dei dati dalla riga di disclaimer #---- dal file.log in esame !!!!!!!!!!!!!
    // per i file prima del 27/10/2021 non sono presente i voltaggi, per il file del 27/10/2021 sta come prima colonna
    t->ReadFile(fileName, "Timestamp/D:Reset_time/I:TRIGGER/I:IN0/I:IN1/I:IN2/I:IN3/I:IN4/I:IN5/I:IN6/I:IN7/I:AND0/I:AND1/I:AND2/I:V_THR1/D:V_THR2/D:V_BIAS/D");
    // Restore previous settings
    gErrorIgnoreLevel = currentIgnoreLevel;
    t->Print();

    //define the variables to be read in the TTree
    double Timestamp, V_bias, V_thr1, V_thr2;
    int IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7;
    int AND0, AND1, AND2;
    TH1F *h_IN0 = new TH1F("IN0", "Single rate distributions; Counts/60s; Occorrenze", 100, 0, 400);TH1F *h_IN1 = new TH1F("IN1", "IN1",  100, 0, 400);TH1F *h_IN2 = new TH1F("IN2", "IN2",  100, 0, 400);
    TH1F *h_IN3 = new TH1F("IN3", "IN3", 100, 0, 400);TH1F *h_IN4 = new TH1F("IN4", "IN4", 100, 0, 400);TH1F *h_IN5 = new TH1F("IN5", "IN5",  100, 0, 400);
    TH1F *h_IN6 = new TH1F("IN6", "IN6",  100, 0,400);TH1F *h_IN7 = new TH1F("IN7", "IN7", 100, 0, 400);
    const char* titolo = "Double rate distributions; Counts/60s; Occorrenze";
    TH1F *h_AND0= new TH1F("", titolo,  100, 0, 100);TH1F *h_AND1= new TH1F("", titolo, 100, 0, 100);TH1F *h_AND2= new TH1F("", titolo,  100, 0, 100);

    //Set the proper addresses
    //t->SetBranchAddress("V_BIAS", &V_bias); // solo per la presa dati del 27/10/2021
    t->SetBranchAddress("Timestamp", &Timestamp);
    t->SetBranchAddress("IN0", &IN0);
    t->SetBranchAddress("IN1", &IN1);
    t->SetBranchAddress("IN2", &IN2);
    t->SetBranchAddress("IN3", &IN3);
    t->SetBranchAddress("IN4", &IN4);
    t->SetBranchAddress("IN5", &IN5);
    t->SetBranchAddress("IN6", &IN6);
    t->SetBranchAddress("IN7", &IN7);
    t->SetBranchAddress("AND0", &AND0);
    t->SetBranchAddress("AND1", &AND1);
    t->SetBranchAddress("AND2", &AND2);
    /*t->SetBranchAddress("V_BIAS", &V_bias);
    t->SetBranchAddress("V_THR1", &V_thr1);
    t->SetBranchAddress("V_THR2", &V_thr2);*/
    //Get the number of entries in the TTree
    int n = t->GetEntries();

    //define other convenient variables and counters
    double first_timestamp = 0.;
    double last_timestamp = 0.;
    double aquisition_time = 0.;
    int cont0 = 0;
    int cont1 = 0;
    int cont2 = 0;
    int cont3 = 0;
    int cont4 = 0;
    int cont5 = 0;
    int cont6 = 0;
    int cont7 = 0;
    int cont_and0 = 0;
    int cont_and1 = 0;
    int cont_and2 = 0;

    //Crates the Graph for the plot
    auto *gr0 = new TGraphErrors();
    auto *gr1 = new TGraphErrors();
    auto *gr2 = new TGraphErrors();
    auto *gr3 = new TGraphErrors();
    auto *gr4 = new TGraphErrors();
    auto *gr5 = new TGraphErrors();
    auto *gr6 = new TGraphErrors();
    auto *gr7 = new TGraphErrors();
    auto *gr_and0 = new TGraphErrors();
    auto *gr_and1 = new TGraphErrors();
    auto *gr_and2 = new TGraphErrors();

    double time, time_save;
    //loop on events
    for(int ev=0; ev<=n*10/seconds; ++ev){//*10/seconds
        //read the current event
        t->GetEntry(ev*seconds/10);

        if(ev==0) first_timestamp = Timestamp;
        else if (ev==n*10/seconds) { last_timestamp = Timestamp; aquisition_time = last_timestamp-first_timestamp;}
            //
        if(ev%1000==0)   cout<< "Event #"<< ev*seconds/10<< endl;

        time = (Timestamp-first_timestamp) / 3600.;
        if(time>=0){ time_save = time; }
        else {time = time_save +Timestamp/3600;}
        //fill the Graph
        gr0->SetPoint(ev, time, IN0-cont0);
        //gr0->SetPointError(ev, 0, sqrt(IN0-cont0));
        h_IN0->Fill(IN0-cont0);

        gr1->SetPoint(ev, time, IN1-cont1);
        //gr1->SetPointError(ev, 0, sqrt(IN1-cont1));
        h_IN1->Fill(IN1-cont1);

        gr2->SetPoint(ev, time, IN2-cont2);
        //gr2->SetPointError(ev, 0, sqrt(IN2-cont2));
        h_IN2->Fill(IN2-cont2);

        gr3->SetPoint(ev, time, IN3-cont3);
        //gr2->SetPointError(ev, 0, sqrt(IN2-cont2));
        h_IN3->Fill(IN3-cont3);

        gr4->SetPoint(ev, time, IN4-cont4);
        //gr6->SetPointError(ev, 0, sqrt(IN6-cont6));
        h_IN4->Fill(IN4-cont4);

        gr5->SetPoint(ev, time, IN5-cont5);
        //gr7->SetPointError(ev, 0, sqrt(IN7-cont7));
        h_IN5->Fill(IN5-cont5);

        gr6->SetPoint(ev, time, IN6-cont6);
        //gr6->SetPointError(ev, 0, sqrt(IN6-cont6));
        h_IN6->Fill(IN6-cont6);

        gr7->SetPoint(ev, time, IN7-cont7);
        //gr7->SetPointError(ev, 0, sqrt(IN7-cont7));
        h_IN7->Fill(IN7-cont7);

        gr_and0->SetPoint(ev, time, AND0-cont_and0);
        //gr_and0->SetPointError(ev, 0, sqrt(AND0-cont_and0));
        h_AND0->Fill(AND0-cont_and0);

        gr_and1->SetPoint(ev, time, AND1-cont_and1);
        //gr_and1->SetPointError(ev, 0, sqrt(AND1-cont_and1));
        h_AND1->Fill(AND1-cont_and1);

        gr_and2->SetPoint(ev, time, AND2-cont_and2);
        //gr_and2->SetPointError(ev, 0, sqrt(AND2-cont_and2));
        h_AND2->Fill(AND2-cont_and2);

        if(IN7<cont7) cout<<"Event number: "<<ev*seconds/10<<endl;

        //update counters
        cont0 = IN0;
        cont1 = IN1;
        cont2 = IN2;
        cont3 = IN3;
        cont4 = IN4;
        cont5 = IN5;
        cont6 = IN6;
        cont7 = IN7;
        cont_and0 = AND0;
        cont_and1 = AND1;
        cont_and2 = AND2;
    }
        //calculate the rates
    cout<<"Rate IN0: "<< double(cont0/aquisition_time) <<" Hz\n";
    cout<<"Rate IN1: "<< double(cont1/aquisition_time) <<" Hz\n";
    cout<<"Rate IN2: "<< double(cont2/aquisition_time) <<" Hz\n";
    cout<<"Rate IN3: "<< double(cont3/aquisition_time) <<" Hz\n";
    cout<<"Rate IN4: "<< double(cont4/aquisition_time) <<" Hz\n";
    cout<<"Rate IN5: "<< double(cont5/aquisition_time) <<" Hz\n";
    cout<<"Rate IN6: "<< double(cont6/aquisition_time) <<" Hz\n";
    cout<<"Rate IN7: "<< double(cont7/aquisition_time) <<" Hz\n";
    cout<<"Rate AND0: "<< double(cont_and0/aquisition_time) <<" Hz\n";
    cout<<"Rate AND1: "<< double(cont_and1/aquisition_time) <<" Hz\n";
    cout<<"Rate AND2: "<< double(cont_and2/aquisition_time) <<" Hz\n";

    //Crates the frame for the plot
    auto canvas = new TCanvas("canvas_XLR8","",1000,800);
    canvas->Divide(2,2);
    canvas->cd(1);
    //Bellurie for the plot
    gr0->SetTitle("IN0");
    gr0->SetMarkerStyle(8);
    gr0->SetMarkerColor(kRed); h_IN0->SetLineColor(kRed);
    gr0->SetLineColor(kRed);  h_IN0->SetLineWidth(2);
    gr0->SetMarkerSize(0.5);

    gr1->SetTitle("IN1");
    gr1->SetMarkerStyle(8);
    gr1->SetMarkerColor(kBlue); h_IN1->SetLineColor(kBlue);
    gr1->SetLineColor(kBlue); h_IN1->SetLineWidth(2);
    gr1->SetMarkerSize(0.5);

    gr2->SetTitle("IN2");
    gr2->SetMarkerStyle(8);
    gr2->SetMarkerColor(kBlack);h_IN2->SetLineColor(kBlack);
    gr2->SetLineColor(kBlack);h_IN2->SetLineWidth(2);
    gr2->SetMarkerSize(0.5);

    gr3->SetTitle("IN3");
    gr3->SetMarkerStyle(8);
    gr3->SetMarkerColor(kMagenta);h_IN3->SetLineColor(kMagenta);
    gr3->SetLineColor(kMagenta);h_IN3->SetLineWidth(2);
    gr3->SetMarkerSize(0.5);

    gr4->SetTitle("IN4");
    gr4->SetMarkerStyle(8);
    gr4->SetMarkerColor(kYellow+2);h_IN4->SetLineColor(kYellow+2);
    gr4->SetLineColor(kYellow+2);h_IN4->SetLineWidth(2);
    gr4->SetMarkerSize(0.5);

    gr5->SetTitle("IN5");
    gr5->SetMarkerStyle(8);
    gr5->SetMarkerColor(kCyan);h_IN5->SetLineColor(kCyan);
    gr5->SetLineColor(kCyan);h_IN5->SetLineWidth(2);
    gr5->SetMarkerSize(0.5);

    gr6->SetTitle("IN6");
    gr6->SetMarkerStyle(8);
    gr6->SetMarkerColor(kOrange);h_IN6->SetLineColor(kOrange);
    gr6->SetLineColor(kOrange);h_IN6->SetLineWidth(2);
    gr6->SetMarkerSize(0.5);

    gr7->SetTitle("IN7");
    gr7->SetMarkerStyle(8);
    gr7->SetMarkerColor(kGreen+2);h_IN7->SetLineColor(kGreen+2);
    gr7->SetLineColor(kGreen+2);h_IN7->SetLineWidth(2);
    gr7->SetMarkerSize(0.5);

    auto mg = new TMultiGraph("mg","mg");
    mg->Add(gr0, "PL");
    mg->Add(gr1, "PL");
    mg->Add(gr2, "PL");
    mg->Add(gr3, "PL");
    mg->Add(gr4, "PL");
    mg->Add(gr5, "PL");
    mg->Add(gr6, "PL");
    mg->Add(gr7, "PL");
    mg->Draw("APL");

    const char* Line1 = "Single rate; Time[h]; Counts/";
    const char* Line2 = "6";
    const char* Line3 = "0s";
    char* TotalLine{ new char[strlen(Line1) + strlen(Line2) + strlen(Line3) + 1] };
    TotalLine = strcpy(TotalLine, Line1);
    TotalLine = strcat(TotalLine, Line2);
    TotalLine = strcat(TotalLine, Line3);
    mg->SetTitle(TotalLine);//"Single rate; Time[s]; Counts/10s"
    mg->SetMinimum(0);//mg->SetMaximum(80);
    TLegend* legend_single = new TLegend(0.8,0.7,0.9,0.9);
    legend_single->AddEntry(gr0,"IN0");legend_single->AddEntry(gr1,"IN1");
    legend_single->AddEntry(gr2,"IN2");legend_single->AddEntry(gr3,"IN3");
    legend_single->AddEntry(gr4,"IN4");legend_single->AddEntry(gr5,"IN5");
    legend_single->AddEntry(gr6,"IN6");legend_single->AddEntry(gr7,"IN7");
    legend_single->Draw();
    gPad->SetGrid();

    canvas->cd(2);
    h_IN0->SetFillColor(kRed); h_IN0->SetFillStyle(3002);
    h_IN0->Draw();
    h_IN1->SetFillColor(kBlue); h_IN1->SetFillStyle(3002);
    h_IN1->Draw("same");
    h_IN2->SetFillColor(kBlack); h_IN2->SetFillStyle(3002);
    h_IN2->Draw("same");
    h_IN3->SetFillColor(kMagenta); h_IN3->SetFillStyle(3002);
    h_IN3->Draw("same");
    h_IN4->SetFillColor(kYellow+2); h_IN4->SetFillStyle(3002);
    h_IN4->Draw("same");
    h_IN5->SetFillColor(kCyan); h_IN5->SetFillStyle(3002);
    h_IN5->Draw("same");
    h_IN6->SetFillColor(kOrange); h_IN6->SetFillStyle(3002);
    h_IN6->Draw("same");
    h_IN7->SetFillColor(kGreen+2); h_IN7->SetFillStyle(3002);
    h_IN7->Draw("same");
    TLegend* legend_single_hist = new TLegend(0.8,0.7,0.9,0.9);
    legend_single_hist->AddEntry(h_IN0,"IN0");legend_single_hist->AddEntry(h_IN1,"IN1");
    legend_single_hist->AddEntry(h_IN2,"IN2");legend_single_hist->AddEntry(h_IN3,"IN3");
    legend_single_hist->AddEntry(h_IN4,"IN4");legend_single_hist->AddEntry(h_IN5,"IN5");
    legend_single_hist->AddEntry(h_IN6,"IN6");legend_single_hist->AddEntry(h_IN7,"IN7");
    legend_single_hist->Draw();
    gPad->SetGrid();gStyle->SetOptStat(0);


    canvas->cd(3);

    gr_and0->SetTitle("AND0");
    gr_and0->SetMarkerStyle(8);
    gr_and0->SetMarkerColor(kRed);h_AND0->SetLineColor(kRed);
    gr_and0->SetLineColor(kRed);h_AND0->SetLineWidth(2);
    gr_and0->SetMarkerSize(0.5);

    gr_and1->SetTitle("AND1");
    gr_and1->SetMarkerStyle(8);
    gr_and1->SetMarkerColor(kBlue);h_AND1->SetLineColor(kBlue);
    gr_and1->SetLineColor(kBlue);h_AND1->SetLineWidth(2);
    gr_and1->SetMarkerSize(0.5);

    gr_and2->SetTitle("AND2");
    gr_and2->SetMarkerStyle(8);
    gr_and2->SetMarkerColor(kGreen+2);h_AND2->SetLineColor(kGreen+1);
    gr_and2->SetLineColor(kGreen+2);h_AND2->SetLineWidth(2);
    gr_and2->SetMarkerSize(0.5);

    auto mg_and = new TMultiGraph("mg_and","mg_and");
    mg_and->Add(gr_and0, "PL");
    mg_and->Add(gr_and1, "PL");
    mg_and->Add(gr_and2, "PL");
    mg_and->Draw("APL");
    mg_and->SetTitle("Double rate; Time[h]; Counts/60s");
    TLegend* legend_double = new TLegend(0.8,0.7,0.9,0.9);
    legend_double->AddEntry(gr_and0,"AND0");legend_double->AddEntry(gr_and1,"AND1");legend_double->AddEntry(gr_and2,"AND2");
    legend_double->Draw();
    gPad->SetGrid();

    canvas->cd(4);
    h_AND0->SetFillColor(kRed); h_AND0->SetFillStyle(3002);
    h_AND0->Draw();
    h_AND1->SetFillColor(kBlue); h_AND1->SetFillStyle(3002);
    h_AND1->Draw("same");
    h_AND2->SetFillColor(kGreen+2); h_AND2->SetFillStyle(3002);
    h_AND2->Draw("same");
    TLegend* legend_double_hist = new TLegend(0.8,0.7,0.9,0.9);
    legend_double_hist->AddEntry(h_AND0,"AND0");legend_double_hist->AddEntry(h_AND1,"AND1");legend_double_hist->AddEntry(h_AND2,"AND2");
    legend_double_hist->Draw();
    gPad->SetGrid(); gStyle->SetOptStat(0);

}


void analysis_voltages(char* fileName, int seconds=10){

    // Silence warnings during TTree::ReadFile
    Int_t currentIgnoreLevel = gErrorIgnoreLevel;
    gErrorIgnoreLevel = kError;
    //read the file and access the TTree
    TTree *t = new TTree("t", "Histogram");
    // !!!!!! Verificare struttura dei dati dalla riga di disclaimer #---- dal file.log in esame !!!!!!!!!!!!!
    // per i file prima del 27/10/2021 non sono presente i voltaggi, per il file del 27/10/2021 sta come prima colonna
    t->ReadFile(fileName, "Timestamp/D:Reset_time/I:TRIGGER/I:IN0/I:IN1/I:IN2/I:IN3/I:IN4/I:IN5/I:IN6/I:IN7/I:AND0/I:AND1/I:AND2/I:V_THR1/D:V_THR2/D:V_BIAS/D");
    // Restore previous settings
    gErrorIgnoreLevel = currentIgnoreLevel;
    t->Print();

    //define the variables to be read in the TTree
    double Timestamp, V_bias, V_thr1, V_thr2;
    int IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7;
    int AND0, AND1, AND2;

    //Set the proper addresses
    //t->SetBranchAddress("V_BIAS", &V_bias); // solo per la presa dati del 27/10/2021
    t->SetBranchAddress("Timestamp", &Timestamp);
    t->SetBranchAddress("IN0", &IN0);
    t->SetBranchAddress("IN1", &IN1);
    t->SetBranchAddress("IN2", &IN2);
    t->SetBranchAddress("IN3", &IN3);
    t->SetBranchAddress("IN4", &IN4);
    t->SetBranchAddress("IN5", &IN5);
    t->SetBranchAddress("IN6", &IN6);
    t->SetBranchAddress("IN7", &IN7);
    t->SetBranchAddress("AND0", &AND0);
    t->SetBranchAddress("AND1", &AND1);
    t->SetBranchAddress("AND2", &AND2);
    t->SetBranchAddress("V_THR1", &V_thr1);
    t->SetBranchAddress("V_THR2", &V_thr2);
    t->SetBranchAddress("V_BIAS", &V_bias);

    //Get the number of entries in the TTree
    int n = t->GetEntries();

    //define other convenient variables and counters
    double first_timestamp = 0.;
    int cont0 = 0;
    int cont1 = 0;
    int cont2 = 0;
    int cont3 = 0;
    int cont4 = 0;
    int cont5 = 0;
    int cont6 = 0;
    int cont7 = 0;
    int cont_and0 = 0;
    int cont_and1 = 0;
    int cont_and2 = 0;

    double const UP_CONT = 170;
    double const LOW_CONT = 0;
    int const BINS_CONT = 50;
    double const UP_VBIAS = 31.5;
    double const LOW_VBIAS = 30;
    int const BINS_VBIAS = 16;

    //Crates the Graph for the plot
    auto *gr0 = new TH2F("", "IN0; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr1 = new TH2D("", "IN1; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr2 = new TH2D("", "IN2; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr3 = new TH2D("", "IN3; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr4 = new TH2D("", "IN4; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr5 = new TH2D("", "IN5; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr6 = new TH2D("", "IN6; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr7 = new TH2D("", "IN7; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr_and0 = new TH2D("", "AND0; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr_and1 = new TH2D("", "AND1; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr_and2 = new TH2D("", "AND2; V_bias [V]; Conteggi ", BINS_VBIAS, LOW_VBIAS, UP_VBIAS, BINS_CONT, LOW_CONT , UP_CONT);
    auto *gr_Vbias = new TGraphErrors();
    auto *gr_V_thr1 = new TGraphErrors();
    auto *gr_V_thr2 = new TGraphErrors();
    auto *h_Vbias = new TH1D("", "Distribuzione Vbias ;V_bias [V]; Conteggi", BINS_VBIAS, LOW_VBIAS, UP_VBIAS);
    auto *h_V_thr1 = new TH1D("", "Distribuzione V_thr1 ;V_thr1 [V]; Conteggi", BINS_VBIAS, -170, -130);
    auto *h_V_thr2 = new TH1D("", "Distribuzione V_thr2 ;V_thr2 [V]; Conteggi", BINS_VBIAS, -180, -140);

    double time, time_save;
    //loop on events
    for(int ev=0; ev<=n*10/seconds; ++ev){//*10/seconds
        //read the current event
        t->GetEntry(ev*seconds/10);

        if(ev==0) first_timestamp = Timestamp;
        if(ev%1000==0)   cout<< "Event #"<< ev*seconds/10<< endl;
        time = (Timestamp-first_timestamp) / 3600.;
        if(time>=0){ time_save = time; }
        else {time = time_save +Timestamp/3600;}
        //fill the Graph
        gr0->Fill( V_bias, IN0-cont0);

        gr1->Fill( V_bias, IN1-cont1);

        gr2->Fill( V_bias, IN2-cont2);

        gr3->Fill( V_bias, IN3-cont3);

        gr4->Fill( V_bias, IN4-cont4);

        gr5->Fill( V_bias, IN5-cont5);

        gr6->Fill( V_bias, IN6-cont6);

        gr7->Fill( V_bias, IN7-cont7);

        gr_and0->Fill( V_bias, AND0-cont_and0);

        gr_and1->Fill( V_bias, AND1-cont_and1);

        gr_and2->Fill( V_bias, AND2-cont_and2);

        gr_Vbias->SetPoint(ev, time , V_bias);
        gr_V_thr1->SetPoint(ev, time , V_thr1);
        gr_V_thr2->SetPoint(ev, time , V_thr2);

        h_Vbias->Fill(V_bias);
        h_V_thr1->Fill(V_thr1);
        h_V_thr2->Fill(V_thr2);

        if(IN7<cont7) cout<<"Event number: "<<ev*seconds/10<<endl;

        //update counters
        cont0 = IN0;
        cont1 = IN1;
        cont2 = IN2;
        cont3 = IN3;
        cont4 = IN4;
        cont5 = IN5;
        cont6 = IN6;
        cont7 = IN7;
        cont_and0 = AND0;
        cont_and1 = AND1;
        cont_and2 = AND2;

    }

    //Crates the frame for the plot
    auto canvas1_1 = new TCanvas("canvas1_XLR8","",1400,600);
    canvas1_1->Divide(4,3);
    canvas1_1->cd(1);
    gr0->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(2);
    gr1->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(3);
    gr2->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(4);
    gr3->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(5);
    gr4->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(6);
    gr5->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(7);
    gr6->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(8);
    gr7->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(9);
    gr_and0->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(10);
    gr_and1->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    canvas1_1->cd(11);
    gr_and2->Draw("zcolor");
    gStyle->SetOptStat("e");
    gPad->SetGrid();
    gPad->BuildLegend();

    //canvas1_1->DrawClone();

    auto canvas1_2 = new TCanvas("canvas2_XLR8","",800,600);
    canvas1_2->cd();
    TPad *pad1 = new TPad("Tensione/Tempo","",0.01,0.02,0.70,0.97);
    TPad *pad2 = new TPad("Distribuzioni in tensione","",0.63,0.02,0.98,0.97);
    pad1->Draw();pad2->Draw();

    pad1->cd();//->Draw();
    TPad *pad11 = new TPad("Vbias","",0.05,0.67,0.95,0.97);
    TPad *pad12 = new TPad("V_thr1","",0.05,0.34,0.95,0.64);
    TPad *pad13 = new TPad("V_thr2","",0.05,0.01,0.95,0.31);
    pad11->Draw();pad12->Draw();pad13->Draw();

    pad11->cd();
    gr_Vbias->SetTitle("V_bias");
    gr_Vbias->SetMarkerStyle(8);
    gr_Vbias->SetMarkerColor(kRed);
    gr_Vbias->SetLineColor(kRed);
    gr_Vbias->SetMarkerSize(0.5);
    gr_Vbias->SetTitle("V_bias; Time[h]; V_bias/60s [V]");
    gr_Vbias->Draw("APL");
    gPad->SetGrid();
     // per la presa dati del 27/10/2021

    pad12->cd();
    gr_V_thr1->SetTitle("V_thr1");
    gr_V_thr1->SetMarkerStyle(8);
    gr_V_thr1->SetMarkerColor(kBlue);
    gr_V_thr1->SetLineColor(kBlue);
    gr_V_thr1->SetMarkerSize(0.5);
    gr_V_thr1->SetTitle("V_thr1; Time[h]; V_thr1/60s [mV]");
    gr_V_thr1->Draw("APL");
    gPad->SetGrid();

    pad13->cd();
    gr_V_thr2->SetTitle("V_thr2");
    gr_V_thr2->SetMarkerStyle(8);
    gr_V_thr2->SetMarkerColor(kGreen);
    gr_V_thr2->SetLineColor(kGreen);
    gr_V_thr2->SetMarkerSize(0.5);
    gr_V_thr2->SetTitle("V_thr2; Time[h]; V_thr2/60s [mV]");
    gr_V_thr2->Draw("APL");
    gPad->SetGrid();
     // per la presa dati del 27/10/2021

    /*auto mg_voltages = new TMultiGraph("mg_voltages","mg_voltages");
    mg_voltages->Add(gr_Vbias, "PL");
    //  // per la presa dati del 27/10/2021
    mg_voltages->Add(gr_V_thr1, "PL");
    mg_voltages->Add(gr_V_thr2, "PL");
    //  // per la presa dati del 27/10/2021
    mg_voltages->Draw("APL");
    mg_voltages->SetTitle("Tensioni di lavoro; Time[h]; Tensione [V]");*/


    pad2->cd();
    TPad *pad21 = new TPad("Vbias","",0.05,0.67,0.95,0.97);
    TPad *pad22 = new TPad("V_thr1","",0.05,0.34,0.95,0.64);
    TPad *pad23 = new TPad("V_thr2","",0.05,0.01,0.95,0.31);
    pad21->Draw(); pad22->Draw(); pad23->Draw();

    pad21->cd();
    h_Vbias->Draw("HIST");
    h_Vbias->SetLineColor(kRed+2);h_Vbias->SetFillColor(kRed); h_Vbias->SetFillStyle(3001);
    gPad->SetGrid();

    pad22->cd();
    h_V_thr1->Draw("HIST");
    h_V_thr1->SetLineColor(kBlue+2);h_V_thr1->SetFillColor(kBlue); h_V_thr1->SetFillStyle(3001);
    gPad->SetGrid();

    pad23->cd();
    h_V_thr2->Draw("HIST");
    h_V_thr2->SetLineColor(kGreen+2); h_V_thr2->SetFillColor(kGreen);h_V_thr2->SetFillStyle(3001);
    gPad->SetGrid();

}
