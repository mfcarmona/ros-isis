
/**** PROGRAMA CLIENTE DEL CBR (CBR_CLIENT.JAVA)*****/

import java.io.*;
import java.net.*;

/**
 * Ejemplo de comunicaci� con el cbr. Utiliza la base de datos cbr_train_all.cbr
 * Lo que hace es mandar dos patrones de entrenamiento al cbr y recibir su respuesta
 * mostrando por pantalla comentarios sobre los resultados obtenidos.
 * Para ejecutarlo, escribir:
 *   java cbr_client [host] [port]
 */
public class cbr_client
{
	protected static InetAddress direccion;
	protected static Socket servidor;
	protected static int num_cliente=0;
	protected static String host;
	protected static int port;
	protected static int inputlength = 8;			// Longitud del vector cbr de entrada
	protected static int outputlength = 4;		// Longitud del vector cbr de salida
			
	
	public static void main(String args[])
	{
		if(args.length<2) // Si no hay par�etros de entrada suficientes
		{
			System.out.println("Use:\n   java cbr_client <host> <port> ");
			//System.exit(0);
			System.out.println("Using default parameters host = local port = 5000");
			host = "127.0.0.1";							// Primer argumento el host
			port = 5000;	// Segundo el puerto
		}
		else
		{
			host = args[0];							// Primer argumento el host
			port = Integer.parseInt(args[1]);	// Segundo el puerto
		}

		try
		{
			servidor=new Socket(host.toString(),port);			// Conecta con el servidor
			System.out.println("\nAccess to the server validated.\n");
			
			// Buffers de entrada y salida de datos
			PrintWriter sout=new PrintWriter( new OutputStreamWriter(servidor.getOutputStream()));
			BufferedReader sin=new BufferedReader( new InputStreamReader(servidor.getInputStream()));
			
			System.out.println("Waiting....");
			try { Thread.sleep(250); }catch(Exception e){}
			
			// Envio el caso 1 de la base de datos cbr_train_all.txt
			String envio1 = "21\n90\n63\n89\n135\n87\n-2000\n-2000\n";
			String envio1show = "21 90 63 89 135 87 -2000 -2000 ";
			sout.write(envio1);
			sout.flush();
			System.out.println("*** ENVIO 1 ***");
			System.out.println("Enviado:  " + envio1show);
			//System.out.println("Deberia recibir: 000 1269  000 00001");			
			//float[] caseInput[] = {21,90,63,89,135,87,-2000,-2000};
			int caseOutput[] = new int[inputlength+outputlength];
			
			System.out.print("Recibido: ");
			for (int i=0;i<(inputlength+outputlength);i++)
			{	// Recibo la respuesta del cbr
				String tmp= sin.readLine();
				caseOutput[i] = (int)Float.parseFloat(tmp);
				System.out.print(caseOutput[i] + " ");
			}
			System.out.println("\n");
			//System.out.println("Y sin embargo recibo: 000 1224  000 00666");
			//System.out.println("Que es el caso 666, parecido al 1 que he puesto, pero no es ese.\n");
			
			// Envio el caso 168 de la base de datos cbr_train_all.txt
			// Envio el caso 1 de la base de datos cbr_train_all.txt
			String envio2 = "12\n110\n29\n90\n70\n89\n161\n84\n";
			String envio2show = "12 110 29 90 70 89 161 84";
			sout.write(envio2);
			sout.flush();
			System.out.println("*** ENVIO 2 ***");
			System.out.println("Enviado:  " + envio2show);
			//System.out.println("Deberia recibir: 000 1269  000 00001");			
			//float[] caseInput[] = {21,90,63,89,135,87,-2000,-2000};
			caseOutput = new int[inputlength+outputlength];
			
			System.out.print("Recibido: ");
			for (int i=0;i<(inputlength+outputlength);i++)
			{	// Recibo la respuesta del cbr
				String tmp= sin.readLine();
				caseOutput[i] = (int)Float.parseFloat(tmp);
				System.out.print(caseOutput[i] + " ");
			}
			System.out.println("\n");
			//System.out.println("Y sin embargo recibo: 000 1955  900 00099");
			//System.out.println("Que es el caso 99, parecido al 168 que he puesto, pero tampoco es ese.");
			servidor.close();
		}
		catch(Exception e)
		{
			System.out.println("Problemas durante la conexi�.");
		}
	}
}

/*
CASOS DE EJEMPLO ENVIADOS AL CBR PARA VER QUE SE OBTEN�:
	
CASE
N 00001
INPUT
-006 3809 4140 5486 6476 1269 1371 4241 3200 2920
ENDINPUT
TRAINEE
ENDTRAINEE
OUTPUTP
 000 1269  000 00001
ENDOUTPUTP
OUTPUTG
ENDOUTPUTG
SOLUTION
ENDSOLUTION
MOREINFO
[T_PF_0001]
ENDMOREINFO
ENDCASE

CASE
N 00168
INPUT
-001 3327 3581 2743 2057 1926 2057 2768 3632 3403
ENDINPUT
TRAINEE
ENDTRAINEE
OUTPUTP
 000 1926  000 00168
ENDOUTPUTP
OUTPUTG
ENDOUTPUTG
SOLUTION
ENDSOLUTION
MOREINFO
[T_PF_0003]
ENDMOREINFO
ENDCASE

CASOS DE EJEMPLO RECIBIDOS AL ENVIAR LOS CASOS ANTERIRES RESPECTIVAMENTE:

CASE
N 00666
INPUT
-002 3809 3962 4952 2793 1224 1305 4597 3378 2946
ENDINPUT
TRAINEE
ENDTRAINEE
OUTPUTP
 000 1224  000 00666
ENDOUTPUTP
OUTPUTG
ENDOUTPUTG
SOLUTION
ENDSOLUTION
MOREINFO
[T_HU_0008]
ENDMOREINFO
ENDCASE

CASE
N 00099
INPUT
-001 4902 4902 4952 3759 3124 3124 3174 2387 1955
ENDINPUT
TRAINEE
ENDTRAINEE
OUTPUTP
 000 1955  900 00099
ENDOUTPUTP
OUTPUTG
ENDOUTPUTG
SOLUTION
ENDSOLUTION
MOREINFO
[T_PF_0002]
ENDMOREINFO
ENDCASE



*/
