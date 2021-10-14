void analysis(char* fileName){

    // Silence warnings during TTree::ReadFile
    Int_t currentIgnoreLevel = gErrorIgnoreLevel;
    gErrorIgnoreLevel = kError;
    //read the file and access the TTree
    TTree *t = new TTree("t", "Histogram");
    t->ReadFile(fileName, "Timestamp/D:kalmanX/D:kalmanY/D:gyroXangle/D:AccXangle/D:CFangleX/D:gyroYangle/D:AccYangle/D:CFangleY/D");
    // Restore previous settings
    gErrorIgnoreLevel = currentIgnoreLevel;
    t->Print();

    //define the variables to be read in the TTree
    double Timestamp;
    double kalmanX, kalmanY, gyroXangle, AccXangle, CFangleX, gyroYangle, AccYangle, CFangleY;


    //Set the proper addresses
    t->SetBranchAddress("Timestamp", &Timestamp);
    t->SetBranchAddress("kalmanX", &kalmanX);
    t->SetBranchAddress("kalmanY", &kalmanY);
    t->SetBranchAddress("gyroXangle", &gyroXangle);
    t->SetBranchAddress("AccXangle", &AccXangle);
    t->SetBranchAddress("CFangleX", &CFangleX);
    t->SetBranchAddress("gyroYangle", &gyroYangle);
    t->SetBranchAddress("AccYangle", &AccYangle);
    t->SetBranchAddress("CFangleY", &CFangleY);

    //Get the number of entries in the TTree
    int n = t->GetEntries();

    //Crates the Graph for the plot
    auto *graph_kalmanX = new TGraph();
    auto *graph_gyroXangle = new TGraph();
    auto *graph_AccXangle = new TGraph();
    auto *graph_CFangleX = new TGraph();

    auto *graph_kalmanY = new TGraph();
    auto *graph_gyroYangle = new TGraph();
    auto *graph_AccYangle = new TGraph();
    auto *graph_CFangleY = new TGraph();

    //loop on events
    for(int ev=0; ev<=n; ++ev){
        //read the current event
        t->GetEntry(ev);

        if(ev%1000==0)   cout<< "Event #"<< ev << endl;

        //fill the Graph
        graph_kalmanX->SetPoint(ev, ev*0.02 , kalmanX );
        graph_gyroXangle->SetPoint(ev, ev*0.02 , gyroXangle );
        graph_AccXangle->SetPoint(ev, ev*0.02 , AccXangle );
        graph_CFangleX->SetPoint(ev, ev*0.02 , CFangleX);

        graph_kalmanY->SetPoint(ev, ev*0.02 , kalmanY );
        graph_gyroYangle->SetPoint(ev, ev*0.02 , gyroYangle );
        graph_AccYangle->SetPoint(ev, ev*0.02 , AccYangle );
        graph_CFangleY->SetPoint(ev, ev*0.02 , CFangleY );

    }

    //Crates the frame for the plot
    auto canvas = new TCanvas("canvas","",1000,600);
    canvas->Divide(2,1);
    canvas->cd(1);
    //Bellurie for the plot
    graph_kalmanX->SetTitle("kalmanX");
    graph_kalmanX->SetMarkerStyle(8);
    graph_kalmanX->SetMarkerColor(kRed);
    graph_kalmanX->SetLineColor(kRed);
    graph_kalmanX->SetMarkerSize(0.5);

    graph_gyroXangle->SetTitle("gyroXangle");
    graph_gyroXangle->SetMarkerStyle(8);
    graph_gyroXangle->SetMarkerColor(kBlue);
    graph_gyroXangle->SetLineColor(kBlue);
    graph_gyroXangle->SetMarkerSize(0.5);

    graph_AccXangle->SetTitle("AccXangle");
    graph_AccXangle->SetMarkerStyle(8);
    graph_AccXangle->SetMarkerColor(kBlack);
    graph_AccXangle->SetLineColor(kBlack);
    graph_AccXangle->SetMarkerSize(0.5);

    graph_CFangleX->SetTitle("CFangleX");
    graph_CFangleX->SetMarkerStyle(8);
    graph_CFangleX->SetMarkerColor(kOrange);
    graph_CFangleX->SetLineColor(kOrange);
    graph_CFangleX->SetMarkerSize(0.5);

    auto mg_x = new TMultiGraph("mg_x","mg_x");
    mg_x->Add(graph_kalmanX, "L");
    // scommentare per vedere gli andamenti degli altri angoli in X
    //mg_x->Add(graph_gyroXangle, "L");
    //mg_x->Add(graph_AccXangle, "L");
    //mg_x->Add(graph_CFangleX, "L");
    mg_x->Draw("AL");

    mg_x->SetTitle("Angolo X; Time[s]; Degrees");

    gPad->BuildLegend();
    gPad->SetGrid();

    canvas->cd(2);

    graph_kalmanY->SetTitle("kalmanY");
    graph_kalmanY->SetMarkerStyle(8);
    graph_kalmanY->SetMarkerColor(kRed);
    graph_kalmanY->SetLineColor(kRed);
    graph_kalmanY->SetMarkerSize(0.5);

    graph_gyroYangle->SetTitle("gyroYangle");
    graph_gyroYangle->SetMarkerStyle(8);
    graph_gyroYangle->SetMarkerColor(kBlue);
    graph_gyroYangle->SetLineColor(kBlue);
    graph_gyroYangle->SetMarkerSize(0.5);

    graph_AccYangle->SetTitle("AccYangle");
    graph_AccYangle->SetMarkerStyle(8);
    graph_AccYangle->SetMarkerColor(kBlack);
    graph_AccYangle->SetLineColor(kBlack);
    graph_AccYangle->SetMarkerSize(0.5);

    graph_CFangleY->SetTitle("CFangleY");
    graph_CFangleY->SetMarkerStyle(8);
    graph_CFangleY->SetMarkerColor(kOrange);
    graph_CFangleY->SetLineColor(kOrange);
    graph_CFangleY->SetMarkerSize(0.5);

    auto mg_y = new TMultiGraph("mg_y","mg_y");
    mg_y->Add(graph_kalmanY, "L");
    //mg_y->Add(graph_gyroYangle, "L");
    //mg_y->Add(graph_AccYangle, "L");
    //mg_y->Add(graph_CFangleY, "L");
    mg_y->Draw("AL");

    mg_y->SetTitle("Angolo Y; Time[s]; Degrees");

    gPad->BuildLegend();
    gPad->SetGrid();

}
