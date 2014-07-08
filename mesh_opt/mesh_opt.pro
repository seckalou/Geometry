#-------------------------------------------------
#
# Project created by QtCreator 2013-08-28T16:49:24
#
#-------------------------------------------------

#QT       += core

#QT       -= gui

TARGET = mesh_opt
#CONFIG   += console
#CONFIG   -= app_bundle

#TEMPLATE = app


SOURCES += main.cpp

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../trimesh2/lib.Linux64/release/ -ltrimesh
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../trimesh2/lib.Linux64/debug/ -ltrimesh
else:unix: LIBS += -L$$PWD/../../trimesh2/lib.Linux64/ -ltrimesh

INCLUDEPATH += $$PWD/../../trimesh2/include
DEPENDPATH += $$PWD/../../trimesh2/include

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../trimesh2/lib.Linux64/release/trimesh.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../trimesh2/lib.Linux64/debug/trimesh.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../trimesh2/lib.Linux64/libtrimesh.a

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../SuiteSparse/CHOLMOD/Lib/release/ -lcholmod
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../SuiteSparse/CHOLMOD/Lib/debug/ -lcholmod
else:unix: LIBS += -L$$PWD/../../SuiteSparse/CHOLMOD/Lib/ -lcholmod

INCLUDEPATH += $$PWD/../../SuiteSparse/CHOLMOD/Include
DEPENDPATH += $$PWD/../../SuiteSparse/CHOLMOD/Include

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../SuiteSparse/CHOLMOD/Lib/release/cholmod.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../SuiteSparse/CHOLMOD/Lib/debug/cholmod.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../SuiteSparse/CHOLMOD/Lib/libcholmod.a

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../SuiteSparse/SuiteSparse_config/release/ -lsuitesparseconfig
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../SuiteSparse/SuiteSparse_config/debug/ -lsuitesparseconfig
else:unix: LIBS += -L$$PWD/../../SuiteSparse/SuiteSparse_config/ -lsuitesparseconfig

INCLUDEPATH += $$PWD/../../SuiteSparse/SuiteSparse_config
DEPENDPATH += $$PWD/../../SuiteSparse/SuiteSparse_config

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../SuiteSparse/SuiteSparse_config/release/suitesparseconfig.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../SuiteSparse/SuiteSparse_config/debug/suitesparseconfig.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../SuiteSparse/SuiteSparse_config/libsuitesparseconfig.a


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../SuiteSparse/COLAMD/Lib/release/ -lcolamd
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../SuiteSparse/COLAMD/Lib/debug/ -lcolamd
else:unix: LIBS += -L$$PWD/../../SuiteSparse/COLAMD/Lib/ -lcolamd

INCLUDEPATH += $$PWD/../../SuiteSparse/COLAMD/Include
DEPENDPATH += $$PWD/../../SuiteSparse/COLAMD/Include

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../SuiteSparse/COLAMD/Lib/release/colamd.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../SuiteSparse/COLAMD/Lib/debug/colamd.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../SuiteSparse/COLAMD/Lib/libcolamd.a



LIBS += -lgomp -lblas -llapack


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../SuiteSparse/AMD/Lib/release/ -lamd
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../SuiteSparse/AMD/Lib/debug/ -lamd
else:unix: LIBS += -L$$PWD/../../SuiteSparse/AMD/Lib/ -lamd

INCLUDEPATH += $$PWD/../../SuiteSparse/AMD/Include
DEPENDPATH += $$PWD/../../SuiteSparse/AMD/Include

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../SuiteSparse/AMD/Lib/release/amd.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../SuiteSparse/AMD/Lib/debug/amd.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../SuiteSparse/AMD/Lib/libamd.a
