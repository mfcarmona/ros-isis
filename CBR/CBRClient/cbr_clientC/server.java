import java.io.* ;
import java.net.* ;

class server
{
	static final int PUERTO=5000;
	public server( )
	{
		try
		{
			ServerSocket servidor = new ServerSocket( PUERTO );
			System.out.println("Escucho el puerto " + PUERTO );
			for ( int numCli = 0; numCli < 3; numCli++ )
			{
				System.out.println("Esperando nuevos clientes...");
				Socket client = servidor.accept(); // Crea objeto
				String tmp = "";
				
				int data = 1;
				while (tmp != null )
				{
					System.out.println("Sirvo al cliente " + numCli);
					System.out.println("Leyendo... ");								
					BufferedReader sin=new BufferedReader( new InputStreamReader(client.getInputStream()) );
					tmp = (String)sin.readLine();
					if (tmp != null)
					{
						tmp = tmp.trim();		
						//InputStream auxIn = client.getInputStream();
						//System.out.println("Leido" + auxIn.toString());
						//DataInputStream flujoIn = new DataInputStream( auxIn );
						//System.out.println( "Recibido: " + flujoIn.readUTF() );
						System.out.println( Integer.toString(data++) + ") Recibido: " + tmp );
						
						//OutputStream auxOut = client.getOutputStream();
						//DataOutputStream flujoOut = new DataOutputStream( auxOut );
						//flujoOut.writeUTF( "Hola cliente " + numCli );
						PrintWriter sout = new PrintWriter( new OutputStreamWriter(client.getOutputStream()) );
						sout.write("Hola cliente " + numCli + "\n");
						sout.flush();
					}
				}
				System.out.println("Cliente cerrado.");
				client.close();				
			}
			System.out.println("Demasiados clientes por hoy");
		}
		catch( Exception e )
		{
			System.out.println( "Excepción: " + e.getMessage() );
			e.printStackTrace();
		}
	}

	public static void main( String[] arg )
	{
		new server();
	}

}

