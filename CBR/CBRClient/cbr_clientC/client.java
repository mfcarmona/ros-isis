import java.io.*;
import java.net.*;

class client
{
	static final String HOST = "localhost";

	static final int PUERTO=5000;

	public client( )
	{
		try
		{
			Socket client = new Socket( HOST , PUERTO );
			OutputStream auxOut = client.getOutputStream();

			PrintWriter sout = new PrintWriter( new OutputStreamWriter(client.getOutputStream()) );
			sout.write("Cliente anonimo.\n");
			sout.flush();
			System.out.println("Enviado: Cliente anonimo.");

			
			BufferedReader sin=new BufferedReader( new InputStreamReader(client.getInputStream()) );
			String tmp = ((String)sin.readLine()).trim();
			System.out.println( "Recibido: " + tmp );			
			client.close();
		}
		catch( Exception e )
		{
			System.out.println( e.getMessage() );
		}
	}

	public static void main( String[] arg )
	{
		new client();
	}

}

