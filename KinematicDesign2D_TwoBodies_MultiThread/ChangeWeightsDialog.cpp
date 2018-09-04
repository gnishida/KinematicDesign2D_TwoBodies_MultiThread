#include "ChangeWeightsDialog.h"

ChangeWeightsDialog::ChangeWeightsDialog(QWidget *parent) : QDialog(parent) {
	ui.setupUi(this);

	ui.lineEditStdDevPositionFirst->setText("0");
	ui.lineEditStdDevOrientationFirst->setText("0");
	ui.lineEditStdDevPositionMiddle->setText("0");
	ui.lineEditStdDevOrientationMiddle->setText("0");
	ui.lineEditStdDevPositionLast->setText("0");
	ui.lineEditStdDevOrientationLast->setText("0");
	ui.lineEditPositionErrorWeight->setText("1");
	ui.lineEditOrientationErrorWeight->setText("5");
	ui.lineEditLinkageLocationWeight->setText("10");
	ui.lineEditTrajectoryWeight->setText("1");
	ui.lineEditSizeWeight->setText("1");
	ui.lineEditNumParticles->setText("100");
	ui.lineEditNumIterations->setText("0");
	ui.checkBoxRecordFile->setChecked(false);

	connect(ui.pushButtonOK, SIGNAL(clicked()), this, SLOT(onOK()));
	connect(ui.pushButtonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));
}

ChangeWeightsDialog::~ChangeWeightsDialog() {
}

void ChangeWeightsDialog::onOK() {
	accept();
}

void ChangeWeightsDialog::onCancel() {
	reject();
}
