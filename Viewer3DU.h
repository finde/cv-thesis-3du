/*
 * Viewer3DU.h
 *
 *  Created on: Feb 17, 2015
 *      Author: morris
 */

#ifndef VIEWER3DU_H_
#define VIEWER3DU_H_

#include <wx/wx.h>

#include "wxViewer.h"

class MyApp: public wxApp
{
	virtual bool OnInit();

public:
	wxFrame *frame;
	wxViewer * glPane;
};

#endif /* VIEWER3DU_H_ */
