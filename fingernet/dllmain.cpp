//
// dllmain.cpp
//

#include <Windows.h>
#include <string>
//#include "resource.h"
//#include "java_api.h"

//#include <passport_uv_static/antheus_passport_check.h>

//#pragma comment(lib, "D:\\Projetos\\cpp\\visa_verification_cmake_vsbuild\\visa_verification_static\\Release\\resources_lib.lib")

BOOL APIENTRY DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
	//if (g_hmod == nullptr && hinstDLL != nullptr)
	//{
	//	g_hmod = hinstDLL;
	//	int r1 = pegar_recurso(IDR_FACEMODELXML, "DATA", face_model_xml, hinstDLL);
	//	int r2 = pegar_recurso(IDR_FACEMODELBIN, "DATA", face_model_bin, hinstDLL);
	//	int r3 = pegar_recurso(IDR_EMBMODELXML, "DATA", embedding_model_xml, hinstDLL);
	//	int r4 = pegar_recurso(IDR_EMBMODELBIN, "DATA", embedding_model_bin, hinstDLL);
	//	int r5 = pegar_recurso(IDR_VISAREFERENCEDB, "DATA", database_dump, hinstDLL);
	//}

	return TRUE;
}