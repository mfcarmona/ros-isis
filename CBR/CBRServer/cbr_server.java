/**
 *
 * Este ejemplo arranca la clase cbrclass creando un servidor cbr para conectarse por socket
 * mostrando el resultado por pantalla.
 * 
 * Por defecto esta clase de ejemplo tiene preparados unos patrones para 
 * la libreria trainLib/SampleTrain.txt, aunque puede usarse.
 * Ademas cuenta el tiempo que tarda en procesar los datos.
 */

import java.io.* ;
import java.net.* ;

class cbr_server
{
	static int PUERTO=5000;
	static cbrclass scbr;											// servidor cbr
	static String libreriaCbr="./trainLib/pruebaCBR.cbr";	// ruta libreria cbr

	public cbr_server()
	{		
		this( new String[0] );
	}

	public cbr_server( String[] args)
	{
		/*
		if (args.length == 0)	System.out.println("No input arguments");
		else
		{
			for (int i = 0; i < args.length; i++)
				System.out.println("arg[" + i + "] = " + args[i]);
		}*/
		//System.out.println("Este programa llama a la clase CBR creando un servidor para mandarle patrones.");
		
		// LEE ARGUMENTOS Y SI NO HAY SE ESTABLECEN LOS PARAMETROS DEL CBR POR DEFECTO
		int inputlength = -1;			// Longitud del vector cbr de entrada
		int outputlength = -1;		// Longitud del vector cbr de salida
		// NOTA: EL VECTOR DEVUELTO POR EL CBR SERA LA UNION DEL VECTOR
		// DE ENTRADA CONCATENADO CON EL DE SALIDA
		if (args.length == 1 || args.length == 2)
		{	// Argumento de entrada libreria cbr a usar
			libreriaCbr = args[0];
			System.out.println("Usando la libreria: " + libreriaCbr);
			inputlength = cbrclass.getNumCbrInputs(libreriaCbr);
			outputlength = cbrclass.getNumCbrOutputs(libreriaCbr);
			if ( args.length == 2 )
			{
				PUERTO = Integer.parseInt(args[1]);
			}
		}
		else
		{	// Si no argumento de entrada, libreria por defecto
			System.err.println ("No se ha especificado ninguna libreria CBR.");
			System.out.println("Por defecto tomada la libreria.cbr \""+ libreriaCbr  + "\"");
			inputlength = cbrclass.getNumCbrInputs(libreriaCbr);
			outputlength = cbrclass.getNumCbrOutputs(libreriaCbr);
		}
		System.out.printf("Num Inputs = %d Num Outputs = %d\n", inputlength, outputlength);
		float[] pesos = new float[inputlength];			// Crea el vector de pesos
		for (int i=0;i<inputlength;i++)		// y lo inicializa a 1
			pesos[i] = 1.0f;
		System.out.print("Vector de pesos: [ ");
		for (int i=0;i<inputlength;i++)		// Se muestran por pantalla
			System.out.print(pesos[i]+" ");
		System.out.println("]");
		// Se crea el servidor CBR, indicandole la libreria a usar, los tamanos de los vectores y el vector de pesos
		scbr = new cbrclass(libreriaCbr,inputlength,outputlength,pesos);	
		if (!scbr.begin())
		{
			System.out.println("Problemas abriendo fichero: " + libreriaCbr + "\nPress enter to end");
			try{System.in.read();}catch(Exception e) {}
			return;
		}
		// FIN CONFIGURACIÓN PARAMETROS CBR
		ServerSocket servidor = null;
		try
		{
			servidor = new ServerSocket( PUERTO );
			System.out.println("Escucho el puerto " + PUERTO );
			int numCli = 0;
			for ( ;;)//int numCli = 0; numCli < 3; numCli++ )
			{
				System.out.println("Esperando nuevos clientes...");
				Socket client = servidor.accept(); // Crea objeto
				String tmp = "";
				numCli++;
				float[] caseInput = new float[inputlength];
				//Float[] caseOutput = new Float[inputlength+outputlength];
				while (tmp != null )
				{
					//System.out.println("Sirvo al cliente " + (numCli));
					//System.out.println("Leyendo... ");								
					boolean showOutput = false;
					//////////////////////// RECEIVE DATA FROM CLIENT /////////////////////////////////////////
					BufferedReader sin=new BufferedReader( new InputStreamReader(client.getInputStream()) );
					if (showOutput)
						System.out.print("<-");
					for (int i=0;i<inputlength;i++)
					{
						tmp = sin.readLine();
						if (tmp != null)
						{
							tmp = tmp.trim();		
							caseInput[i] = Float.parseFloat(tmp);
							if (showOutput)
								System.out.print("[" + caseInput[i] + "]");
						}
					}
					if (showOutput)
						System.out.println("");
					if (tmp != null)
					{
						/////////////////////////// SEND DATA TO CBR AND RECEIVE ANSWER ///////////////////////////////7
						while ( !scbr.dataInReady() ) { try{Thread.sleep(5);}catch(Exception e) {} }
						scbr.dataInSend(caseInput);
						while ( !scbr.dataOutReady() ) { try{Thread.sleep(5);}catch(Exception e) {} }
						float[] caseOutput = scbr.dataOutRead();
						
						/////////////////////////// SHOW ANSWER ON CONSOLE ///////////////////////
						//////////////////////  AND SEND ANSWER TO THE CLIENT ///////////////////////7
						PrintWriter sout = new PrintWriter( new OutputStreamWriter(client.getOutputStream()) );
						if (showOutput)
							System.out.print("->");
						for (int i = 0;i< (inputlength + outputlength);i++)
						{
							if (showOutput)
							{
								if (i == inputlength)
									System.out.print(" - ");
								System.out.print("["+caseOutput[i]+"]");
							}
							sout.write(caseOutput[i] + "\n");
						}
						if (showOutput)
							System.out.println("\n\n");
						sout.flush();
					}
				}
				System.out.println("Cliente cerrado.");
				client.close();				
			}
			
		}
		catch( Exception e )
		{
			System.out.println( "Excepcion: " + e.getMessage() );
			e.printStackTrace();
			try{System.in.read();}catch(Exception e1) {}
		}
		System.out.println("Demasiados clientes por hoy");
		if (servidor != null )
		{
			try {servidor.close();}
			catch (IOException e) {}
		}
		scbr.finCbr();
	}

	public static void main( String[] arg )
	{
		new cbr_server(arg);
	}

}

