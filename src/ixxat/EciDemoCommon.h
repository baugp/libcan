///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2013 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  ECI API Demo common helper functions

  @author Michael Ummenhofer (ummenhofer@ixxat.de)
  @file EciDemoCommon.h
*/

#ifndef __ECIDEMOCOMMON_H__
#define __ECIDEMOCOMMON_H__


//////////////////////////////////////////////////////////////////////////
// include files
#include "ECI_hwtype.h"

#include <ECI_pshpack1.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros

/* _countof helper */
#if !defined(_countof)
  #define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

/** ECI Demo error check macro @ingroup EciDemo */
#define ECIDRV_CHECKERROR(FuncName) \
{\
  if(ECI_OK == hResult)\
  {\
    OS_Printf(#FuncName "...succeeded.\n"); \
  }\
  else\
  {\
    OS_Printf( #FuncName "...failed with error code: 0x%08X. %s\n", \
               hResult, \
               ECIDRV_GetErrorString(hResult)); \
  }\
  OS_Fflush(stdout); \
}
#if !IXXAT_DEBUG
#define ECIDRV_CHECKERROR(FuncName) {}
#endif

//////////////////////////////////////////////////////////////////////////
// data types

#include <ECI_poppack.h>

//////////////////////////////////////////////////////////////////////////
// static function prototypes

//*** C-API
#ifdef __cplusplus
extern "C"
{
#endif

void        EciPrintHwInfo            ( const ECI_HW_INFO*            pstcHwInfo);
void        EciPrintCtrlCapabilities  ( const ECI_CTRL_CAPABILITIES*  pstcCtrlCaps);
void        EciPrintCtrlMessage       ( const ECI_CTRL_MESSAGE*       pstcCtrlMsg);

ECI_RESULT  EciGetNthCtrlOfClass      ( const ECI_HW_INFO*            stcHwInfo,
                                        e_CTRLCLASS                   eCtrlClass,
                                        DWORD                         dwRelCtrlIndex,
                                        DWORD*                        pdwCtrIndex);


//*** C-API
#ifdef __cplusplus
};
#endif


#endif //__ECIDEMOCOMMON_H__