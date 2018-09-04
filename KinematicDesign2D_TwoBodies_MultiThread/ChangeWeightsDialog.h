#ifndef CHANGEWEIGHTSDIALOG_H
#define CHANGEWEIGHTSDIALOG_H

#include <QDialog>
#include "ui_ChangeWeightsDialog.h"

class ChangeWeightsDialog : public QDialog {
	Q_OBJECT

public:
	Ui::ChangeWeightsDialog ui;

public:
	ChangeWeightsDialog(QWidget *parent = 0);
	~ChangeWeightsDialog();

public slots:
	void onOK();
	void onCancel();
};

#endif // CHANGEWEIGHTSDIALOG_H
