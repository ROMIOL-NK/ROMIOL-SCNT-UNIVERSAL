#ifndef AUTOCLONE_H
#define AUTOCLONE_H

#include "Objects/objects.h"
#include "WorkFlow/stepbuildmap.h"
#include "WorkFlow/stepfetchoocyte.h"
#include "WorkFlow/steprotateoutplane.h"
#include "WorkFlow/steprotateinplane.h"
#include "WorkFlow/steppenetrate.h"
#include "WorkFlow/stepenucleation.h"
#include "WorkFlow/stepexpel.h"
#include "WorkFlow/stepfetchsoma.h"
#include "WorkFlow/stepfetchoocyteforsoma.h"
#include "WorkFlow/steprotateforsoma.h"
#include "WorkFlow/steppenetrateforsoma.h"
#include "WorkFlow/stepinjectsoma.h"
#include "WorkFlow/stepreleaseoocyte.h"
#include "WorkFlow/stepexception.h"
class AutoClone: public QObject
{
    Q_OBJECT
public:
    AutoClone();

    bool ON;

public slots:
    bool run(WORKSTEP arg_nextStep=BUILDMAP);
signals:
    void finish();
};

#endif // AUTOCLONE_H
