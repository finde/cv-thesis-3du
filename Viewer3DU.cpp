//============================================================================
// Name        : main.cpp
// Author      : Morris
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "Viewer3DU.h"
#include "wxSidePanel.h"

using namespace std;

// Main is hidden in this wxMacro!
IMPLEMENT_APP_CONSOLE(MyApp)

bool MyApp::OnInit()
{
	frame = new wxFrame((wxFrame *)NULL, -1,  wxT("3DUniversum viewer"), wxPoint(50,50), wxSize(800,600));
	wxBoxSizer *hbox = new wxBoxSizer(wxHORIZONTAL);

	int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};
	glPane = new wxViewer( (wxFrame*) frame, args);

	wxSidePanel *m_rp = new wxSidePanel(frame, this);

	hbox->Add(m_rp, 0, wxEXPAND | wxALL, 1);
	hbox->Add(glPane, 1, wxEXPAND | wxALL);

	frame->SetSizer(hbox);
	frame->SetAutoLayout(true);
	frame->Show();
	return true;
}
