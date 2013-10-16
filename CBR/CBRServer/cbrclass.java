// Añadido <String> para compilar correctamente en java 1.5
//package cbr;           	// Paquete al que pertenece la clase

import java.io.*;
import java.util.*;
import java.net.*;
//import java.util.NoSuchElementException;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.etools.*;
import upc.lsi.kemlg.cbr.distances.*;
import uma.dte.etools.cbr.distances.*;

/**
 * This class implements the Case-Based Reasoner, the one that replies
 * to all the on-line queries. <br>
 * To do so, first it opens the Case Library
 * file (created by the Case Library Builder) in order to recover the
 * structure of the Case Library. Then it waits for any Socket connections
 * coming from the Servlets inside the HTTP server, one Socket connection
 * per Servlet.
 * <br>
 * The Case-Based Reasoner can efficiently attend all the connections
 * concurrently, as each connection is attended by a JAVA thread (the
 * inner class cbrThread). It can also handle errors in the queries
 * submitted, as it uses the JAVA exception handler mechanism.
 * @see CBR
 * @see cbrThread
 */
public class cbrclass
{

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  /** The Case Library. */
   static EToolsCBR cbr;

  /** The name of the file where information is loaded and/or saved */
   static String fileName;
	static int lengthIn;				// NNN
   static int lengthOut;			// NNN
   static float[] vectorPesos;	// NNN

  /** The attribute table */
   static EToolsAttributeTable atrTab;

	/** PPP Indica estado del servidor [Conectado/Desconectado] */
	static boolean conectado = false;

	/** PPP thread para poder acceder y cambiar parametros internos */
	cbrThread cbrthread;

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================

	static public int getNumCbrInputs(String cbrFile)
	{
		EToolsCBR tmpCbr;
      try
      {
			ObjectInputStream fichero = new ObjectInputStream(new FileInputStream(cbrFile));
			tmpCbr = (EToolsCBR)fichero.readObject();
      }
		catch (FileNotFoundException e)
		{
			System.out.println("ERROR <El fichero \"" + cbrFile + "\" no existe>");
			return -1;
		}
      catch (Exception e)
      {
	 		e.printStackTrace();
			return -1;
      }

		EToolsAttributeTable tmpAtrTab = (EToolsAttributeTable) tmpCbr.getAttributeTable();
		int tmpLengh = 0;
		String tmpParam = new String("In"+tmpLengh);
		//System.out.printf("Accediendo a %d\n", tmpLengh);
		AttributeDef tmpAttrDef = (AttributeDef) tmpAtrTab.get(tmpParam);
		while(tmpAttrDef != null)
		{	
			++tmpLengh;
			tmpParam = new String("In"+tmpLengh);
			//System.out.printf("Accediendo a %d\n", tmpLengh);
			tmpAttrDef = (AttributeDef) tmpAtrTab.get(tmpParam);
		}
		//System.out.printf("Num Inputs = %d\n", tmpLengh);
		return (tmpLengh);
	}

	static public int getNumCbrOutputs(String cbrFile)
	{
		EToolsCBR tmpCbr;
      try
      {
			ObjectInputStream fichero = new ObjectInputStream(new FileInputStream(cbrFile));
			tmpCbr = (EToolsCBR)fichero.readObject();
      }
		catch (FileNotFoundException e)
		{
			System.out.println("ERROR <El fichero \"" + cbrFile + "\" no existe>");
			return -1;
		}
      catch (Exception e)
      {
	 		e.printStackTrace();
			return -1;
      }

		EToolsAttributeTable tmpAtrTab = (EToolsAttributeTable) tmpCbr.getAttributeTable();
		int tmpLengh = 0;
		String tmpParam = new String("Out"+tmpLengh);
		//System.out.printf("Accediendo a %d\n", tmpLengh);
		AttributeDef tmpAttrDef = (AttributeDef) tmpAtrTab.get(tmpParam);
		while(tmpAttrDef != null)
		{	
			++tmpLengh;
			tmpParam = new String("Out"+tmpLengh);
			//System.out.printf("Accediendo a %d\n", tmpLengh);
			tmpAttrDef = (AttributeDef) tmpAtrTab.get(tmpParam);
		}
		//System.out.printf("Num Outputs = %d\n", tmpLengh);
		return (tmpLengh);
	}

  	/** PPP  pasa los datos al cbr como si se tratase del socket original */
	public void dataInSend(float[] data)	// aaaaaa
	{
		if (data != null)
			cbrthread.dataInSend(data);
		else
			System.out.println("cbrclass::data = null");
	}

	/** Devuelve true si cbr esta listo para recibir un nuevo vector de entrada */
	public boolean dataInReady()
	{
		return (cbrthread.dataInReady());
	}

	/** Devuelve true si el vector de salida del cbr esta listo para ser leido */
	public boolean dataOutReady()
	{
		return (cbrthread.dataOutReady());
	}

	/** PPP  pasa los datos al cbr como si se tratase del socket original */
	public float[] dataOutRead()
	{
		return (cbrthread.dataOutRead());
	}		// aaaaaa


	/** PPP  finaliza el thread principal del cbr */
	public void finCbr()
	{
		cbrthread.finCbr();
	}

   /** PPP CONSTRUCTOR. Parm: Nombre de la libreria CBR a usar */
   public cbrclass(String cbr_library,int lengthIn,int lengthOut,float[] vectorPesos)	// NNN
	{
     this.fileName = cbr_library;
     this.lengthIn = lengthIn;
     this.lengthOut = lengthOut;
     this.vectorPesos = vectorPesos;
   }

  /**
   * Starts the execution of the Case-Based Reasoner. <br>
   * <br>
   * It loads the Case Library from a file and then gets ready to receive
   * requests of information through the specified port.
   * {PRE: x is an instance of Case-Based Reasoner,
   *	   s_port is an available port of the machine where the
   *		  Case-Based Reasoning will run.
   * }<br>
   * {POST: the execution of the reasoner has ended
   * }<br>
   * @param   s_port the port where the Case-Based Reasoner will wait for requests.
   */
  public boolean begin()
  {

      // 1.- carreguem el cbr de disc
      try
      {
			//Abre el fichero de la libreria.
			ObjectInputStream fichero = new ObjectInputStream(
							new FileInputStream(fileName));
			cbr = (EToolsCBR)fichero.readObject();
      }
		catch (FileNotFoundException e)
		{
			System.out.println("ERROR <El fichero \"" + fileName + "\" no existe>");
			return false;
		}
      catch (Exception e)
      {
	 		e.printStackTrace();
			return false;
      }

     atrTab = (EToolsAttributeTable) cbr.getAttributeTable();

/*
		int tmpInLengh = 0;
		String tmpParam = new String("In"+tmpInLengh);
		//System.out.printf("Accediendo a %d\n", tmpInLengh);
		AttributeDef tmpAttrDef = (AttributeDef) atrTab.get(tmpParam);
		while(tmpAttrDef != null)
		{	
			++tmpInLengh;
			tmpParam = new String("In"+tmpInLengh);
			//System.out.printf("Accediendo a %d\n", tmpInLengh);
			tmpAttrDef = (AttributeDef) atrTab.get(tmpParam);
			
		}
		System.out.printf("Num Inputs = %d\n", tmpInLengh);
*/
		for(int i=0;i<lengthIn;i++)
		{	// NNN
			String param = new String("In"+i);
			if (vectorPesos[i] != 1.0f)
     			System.out.println("Modificado peso del parametro " + param + " con peso " + vectorPesos[i]);
			AttributeDef auxDef = (AttributeDef) atrTab.get(param);
			auxDef.setWeight(vectorPesos[i]);
		}
      System.out.println("Cbr creado con éxito");

		cbrthread = new cbrThread(lengthIn, lengthOut, vectorPesos);	// NNN
		return true;
   }



  //====||===============================================================
  //===\||/=============== metodos auxiliares ==========================
  //====\/===============================================================


  /**
   * Shows a dialog to the user to ask the name of the file where the
   * Case Library is stored. <br>
   * <br>
   * {PRE: x is an instance of CBReasoner,
   * }<br>
   * {POST: returns the name of the file where the Case Library will be
   *	    loaded.
   * }<br>
   * @return String the name of the file.
   */
  public String askFileName()
  {
      String nom="";
      Frame f1 = new Frame();

      FileDialog dialog = new FileDialog(f1,"Cargar CBR de...");
      dialog.setVisible(true);
      if (dialog.getFile()!=null)
      {
	 		nom = dialog.getDirectory() + dialog.getFile();
      }

      return nom;
  }



  //====||===============================================================
  //===\||/================= Clases Internas ============================
  //====\/===============================================================

  /**
   * This inner class implements the threads that serve each request
   * received by the Case-Based Reasoner.
   * The use of Java threads lets the Case-Based Reasoner to process
   * severar requests concurrently.
   */
  class cbrThread extends Thread
  {

    //====||=============================================================
    //===\||/=================== atributos ==============================
    //====\/=============================================================

	 // <NNN>
	 // Solo hay que cambiar más partes del codigo aparte de estas en el
	 // caso de que se pongan más parametros de los que hay, para añadirlos
	 // en sus correspondientes lugares
	 // Si sobran parametros, bastaria con borrarlos del codigo
// aaaaaa
	 int inputlength;	/* Longitud del vector de entrada */
    int outputlength;	/* Longitud del vector de salida */
	 float[] valparmOut;

	 float[] data;						// Dato a leer
	 boolean dataInValido=false;	// true => se lee el nuevo dato;
	 //boolean dataInReady=false;	// true => cbr listo para leer nuevo vector entrada
	 boolean fincbr = false;		// true => finaliza el thread y el cbr.
	 boolean dataOutValido=true;	// true => se puede leer la respuesta del CBR. Inicialmente habilitado porque indica que se manden vectores de entrada tambien
	 //String caseOutput;				// Caso que se devuelve como respuesta del CBR

	 //====||=============================================================
    //===\||/=============== metodos secundarios ========================
    //====\/=============================================================

      	/** PPP  pasa los datos al cbr como si se tratase del socket original */
	public void dataInSend(float[] data)
	{
		this.data = data;
		dataInValido = true;
		dataOutValido = false;
	}

	/** Devuelve true si cbr esta listo para recibir un nuevo vector de entrada */
	public boolean dataInReady()
	{
		return (dataOutValido);		// Si el vector de salida esta listo para ser leido es que el cbr esta listo para un nuevo vector de entrada
	}

	/** Devuelve true si el vector de salida del cbr esta listo para ser leido */
	public boolean dataOutReady()
	{
		return (dataOutValido);		// vector de salida esta listo para ser leido
	}

	/** PPP  pasa los datos al cbr como si se tratase del socket original */
	public float[] dataOutRead()
	{
		return (valparmOut);
	}
	// aaaaa

	/** Finaliza el thread */
	public void finCbr()
	{
	 	fincbr = true;
	}

    //====||=============================================================
    //===\||/=============== metodos principales ========================
    //====\/=============================================================

     /**
      * Creates a thread to answer a request. <br>
      * {PRE: request is a Java Socket
      * }<br>
      * {POST: returns an instance of ServerSocket
      * }<br>
      * @param request the socket connection this thread with the Java Servlet in the HTTP server.
      */
     cbrThread(int lengthIn, int lengthOut, float[] vectorPesos)	// NNN
     {
	     	inputlength = lengthIn;						/* Longitud del vector de entrada */
    		outputlength = lengthOut;					/* Longitud del vector de salida */
			valparmOut = new float[inputlength + outputlength];
       	start();
     }


     /**
      * Processes the request received through the socket and then sends
      * back the response obtained from the Case Library. <br>
      * {PRE: x is an isntance of ServerSocket
      * }<br>
      * {POST: the request has been answered
      * }<br>
      */
		public void run ()
		{	// NNN
			String[] parmIn = new String[inputlength];
			String[] parmOut = new String[inputlength + outputlength];
			//System.out.println("inputlength: " + inputlength + " outputlength" + outputlength);
			for(int i = 0; i<inputlength;i++)
			{
				 parmIn[i] = new String("In" + i);
				 parmOut[i] = new String("In" + i);
				 //System.out.println("ParmOut[" + i + "] = " + parmOut[i]);
			}
			for(int i = inputlength; i<(inputlength + outputlength);i++)
			{
				 parmOut[i] = new String("Out" + (i-inputlength) );
				 //System.out.println("ParmOut[" + i + "] = " + parmOut[i]);
			}
			String[] arrDesc;
			float[] arrVal;
			arrDesc = new String[inputlength];
			int j;
			for (j = 0;j<inputlength;j++)
			arrDesc[j] = parmIn[j].toString();

			//boolean finalitzat=false;
			Vector<String> caseQuery, caseResponse;
			int numVars;
			Case currentCase, resCase;
			AttributeVal atrCur, atrRet;
			String name_CBR_lib, name_case;

			String [] caseInput;

			while (!fincbr)
			{

				caseInput = new String[inputlength];
				//caseOutput = "";
				arrVal = new float[inputlength];
				caseQuery = null;
				caseResponse = new Vector<String>();
				currentCase = null;
				resCase = null;
				atrCur = null;
				atrRet = null;
				name_CBR_lib = null;
				name_case = null;

				try
				{// aaaaa
					while (!dataInValido && !fincbr) { try{Thread.sleep(5); } catch(Exception e) {} }
					if (fincbr) break;
					dataInValido = false;	// Una vez leido se pone a false
					for (int i=0;i<inputlength;i++)
					{
						caseInput[i] = String.valueOf(data[i]);
						arrVal[i] = (float)data[i];
					}


					ComposedDistance distAux = cbr.getDistanceMeasure();
					//System.out.println("cbrclass: la distancia usada es " + distAux);
					currentCase = EToolsCase.parseFromArray(atrTab,
										cbr.getDistanceMeasure(),
										arrDesc,
										arrVal,
										inputlength);


					cbr.setCurrentCase(currentCase);
					RetrievedCase retrCase = cbr.retrieve1Case();// Si cbr's distintos, error aquí
					resCase = retrCase.getCase();
					caseResponse.add(retrCase.getIdCase());
					caseResponse.add("" + retrCase.getSimToCurrent());

					EToolsCaseDescription cDesc = (EToolsCaseDescription) currentCase.getCaseDescription();
					Iterator i2 = cDesc.iterator();
					EToolsCaseDescription rDesc = (EToolsCaseDescription) resCase.getCaseDescription();
					for (Iterator i = rDesc.iterator();i.hasNext();)
					{
						atrCur = (AttributeVal)i2.next();
						atrRet = (AttributeVal)i.next();
						caseResponse.add(atrCur.getAttrName());
						caseResponse.add(atrCur.getQualVal());
						if (atrCur.getAttrType()==AttributeDef.LINEAR)
						{
							caseResponse.add("" + ((LinearAttribute)atrCur).getQuantVal());
							caseResponse.add(atrRet.getQualVal());
							caseResponse.add("" + ((LinearAttribute)atrRet).getQuantVal());
						}
						else
						{
							caseResponse.add("----");
							caseResponse.add(atrRet.getQualVal());
							caseResponse.add("----");
						}
					}
					String name = "";

					AttributeValTreeSet cSol =((EToolsCaseSolution) resCase.getCaseSolution()).outputParams;
					for (Iterator i = cSol.iterator();i.hasNext();)
					{
						atrRet = (AttributeVal)i.next();
						name = atrRet.getAttrName();
						for (j = 0;j < (inputlength + outputlength);j++)
						{
							if (name.equals(parmOut[j].toString()))
							{
								 float tmp = ((LinearAttribute) atrRet).getQuantVal();
								 valparmOut[j] = tmp;
								//System.out.println(parmOut[j].toString()+"\t= " + valparmOut[j]);	// BBB
								break;
							}
						}
						if (j == (inputlength + outputlength) )
							System.out.println("ERROR< Parametro desconocido >"+ name);
					}
					dataOutValido = true;//aaaaa

					// Muestra por pantalla los vectores de entrada y salida
					boolean showDebug = false;
					if (showDebug)
					{
						System.out.print(" <- ");
						for (int i=0;i<inputlength;i++)
							System.out.print("[" + (int)arrVal[i] + "]");			// BBB
						System.out.print("\n -> ");
						for (int i=0; i< (inputlength + outputlength); i++)
						{
							if (i == inputlength)
								System.out.print(" - ");
							if ( i < inputlength )
							{
								if ( (int)valparmOut[i] == -2000 )
									System.out.print("[---]");
								else
									System.out.print("[" + (int)valparmOut[i] + "]");
							}
							if ( (i >= inputlength) )
							{
								if ( (i < (inputlength + outputlength -1)) )
									System.out.print("[" + ((valparmOut[i]>1.0f)?1:(valparmOut[i]<-1.0f)?-1:0) + "]");
								else
									System.out.print("[" + (int)valparmOut[i] + "]");
							}
						}
						System.out.println("");
					}
				}
				catch(NoSuchElementException e)
				{
					e.printStackTrace();
				}
	  		}		// Fin while (!fincbr)
		}			// Fin run class cbrThread
	}				// Fin class cbrThread
}

