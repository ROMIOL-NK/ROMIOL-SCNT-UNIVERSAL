#include "autoclone.h"

AutoClone::AutoClone()
{

}
bool AutoClone::run(WORKSTEP arg_nextStep)
{
    ON = true;
    while(ON)
    {
        switch (arg_nextStep) {
        case BUILDMAP:
            arg_nextStep = stepBuildMap->run();
            break;
        case FETCHOOCYTE:
            arg_nextStep = stepFetchOocyte->run();
            break;
        case ROTATEOUTPLANE:
            arg_nextStep = stepRotateOutPlane->run();
            break;
        case ROTATEINPLANE:
            arg_nextStep = stepRotateInPlane->run();
            break;
        case PENETRATE:
            arg_nextStep = stepPenetrate->run();
            break;
        case ENUCLEATION:
            arg_nextStep = stepEnucleation->run();
            break;
        case EXPEL:
            arg_nextStep = stepExpel->run();
            break;
        case FETCHSOMA:
            arg_nextStep = stepFetchSoma->run();
            break;
        case FETCHOOCYTEFORSOMA:
            arg_nextStep = stepFetchOocyteForSoma->run();
            break;
        case ROTATEFORSOMA:
            arg_nextStep = stepRotateForSoma->run();
            break;
        case PENETRATEFORSOMA:
            arg_nextStep = stepPenetrateForSoma->run();
            break;
        case INJECTSOMA:
            arg_nextStep = stepInjectSoma->run();
            break;
        case RELEASEOOCYTE:
            arg_nextStep = stepReleaseOocyte->run();
            break;
        case EXCEPTION:
            arg_nextStep = stepException->run();
            break;
        case END:
            ON = false;
            emit break;
        default:
            break;
        }
    }
    emit finish();
    return true;
}
