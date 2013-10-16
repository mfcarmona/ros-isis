/*
- Poner un apartado de ayuda que cargue el pdf (rehacer el pdf)
- Añadir al método autónomo la opcion de ciclicidad y distancias
- Añadir control por teclas:

public class ControllerGUI extends JFrame implements ActionListener, KeyListener {


	public class EscFilter extends KeyAdapter {
		JButton trigger;
		EscFilter(JButton trigger) { this.trigger=trigger; }
		public void keyReleased(KeyEvent evt) {
			if(evt.getKeyCode()==KeyEvent.VK_ESCAPE)
				trigger.doClick();
		}
	}
	public class DelFilter extends KeyAdapter {
		ControllerGUI gui;
		DelFilter(ControllerGUI gui) { this.gui=gui; }
		public void keyReleased(KeyEvent evt) {
			if(evt.getKeyCode()==KeyEvent.VK_BACK_SPACE || evt.getKeyCode()==KeyEvent.VK_DELETE)
				gui.actionPerformed(new ActionEvent(evt.getSource(),0,"delbookmark"));
		}
	}
	public class ReturnFilter extends KeyAdapter {
		ControllerGUI gui;
		ReturnFilter(ControllerGUI gui) { this.gui=gui; }
		public void keyReleased(KeyEvent evt) {
			if(evt.getKeyCode()==KeyEvent.VK_ENTER)
				gui.doubleClicked((JList)evt.getSource(),0);
		}
	}
	
	public void keyPressed(KeyEvent e) {
	 if (e.getComponent()==inputField) {
		int key=e.getKeyCode();
		if (key==KeyEvent.VK_UP || key==KeyEvent.VK_KP_UP) {
		  inputFieldLocation--;
		  if (inputFieldLocation < 0)
			 inputFieldLocation+=inputFieldHistory.size();
		  inputField.setText((String)inputFieldHistory.get(inputFieldLocation));
		} else if (key==KeyEvent.VK_DOWN || key==KeyEvent.VK_KP_DOWN) {
		  inputFieldLocation++;
		  if (inputFieldLocation >= inputFieldHistory.size())
			 inputFieldLocation-=inputFieldHistory.size();
		  inputField.setText((String)inputFieldHistory.get(inputFieldLocation));
		}
	 }
  }
  public void keyReleased(KeyEvent e) { }
  public void keyTyped(KeyEvent e) { }

	menu.addKeyListener(new ReturnFilter(this));
	inputField.addKeyListener(this);
	scripts.addKeyListener(new ReturnFilter(this));
	addKeyListener(esc);
	inputField.addKeyListener(esc);
	menu.addKeyListener(esc);scripts.addKeyListener(new DelFilter(this));
*/


import javax.swing.*;						// De aquí son todas las clases que llevan 
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.etools.*;
import upc.lsi.kemlg.cbr.distances.*;
import uma.dte.etools.cbr.distances.*;
import java.util.prefs.Preferences; 	// Para salvar la configuración al cerrar 

/**
 * - Esta versión está adaptada para java 1.5. Tiene mensajes tooltip en los menus
 *   aclarando la funcionalidad de los menus.
 * - Tiene anuladas las opciones "crear Index" y "cargar casos actuales" por no 
 *   encontrarse ninguna utilidad en ellas.
 * - Muestra al usuario más mensajes de errores tanto por consola como por pantalla.
 *   Permite también anular los mensajes por pantalla para reducir los
 *   mensajes basura.
 * - Permite la elección de ciclicidad de variables mediante la carga de un fichero
 *   con el nombre de las variables que se establecerán como cíclicicas (un nombre
 *   por línea del archivo).
 * - Permite escoger de forma gráfica la distancia a usar a la hora de crear la 
 *   librería CBR.
 * - Tiene una opción para crear una librería CBR directamente (carga de archivos)
 *   y guardado de la librería directamente.
 * - Si al invocar al constructor se le pasan los nombres de los archivos de 
 *   definición de variables y de datos creará automáticamente la librería CBR
 *   correspondiente sin mostrar ningún interfaz gráfico. Habría que permitir la
 *   elección de ciclicidad y distancia mediante consola también, pero aún no está
 *   disponible.
 */

/**
 * This class implements the Case Library Builder, the component
 * of the CBR Server that manages the off-line 
 * input of information. It parses the precomputed information 
 * to transform each simulation into a case, which is then properly
 * added to the case library. To interpret part of the information
 * of the simulation, the Case Library Builder needs also the 
 * descriptions of all the parameters that appear in the simulation
 * data. Both the Simulated Emergencies Data file and the
 * RTXPS Parameter Descriptors file are received either through 
 * e-mail or through a FTP session.  When the Case Library has been
 * built, then it is stored in a file.
 * It also allows to load a stored Case Library to make further changes 
 * to it, or to add more off-line information.
 * @see CBR
 */
public class CBRCompiler extends JFrame implements ActionListener
{
  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  //------------------ variables de pantalla -----------------------

  static Preferences prefs = Preferences.userNodeForPackage(CBRCompiler.class);
  private final static long serialVersionUID = 100;
	/** A GUI's text field to display messages */
	JTextField tMessages;
	
	/** A GUI's text area */
	JTextArea  tVar, tCurrCases, tCbr; 
	
	/** A GUI's label */ 
	JLabel lMessages, lVar, lCurrCases, lCbr;
	
	/** A GUI's menu bar */ 
	JMenuBar mBar;
	
	/** A GUI's menu */ 
	JMenu mFile, mCrearlibreriaCBR, mCiclicidad, mDistancia, mOpciones, mHelp;
	
	/** A GUI's menu option */ 
	JMenuItem mNew, mLoad, mCrearCBR, /*mIndexFile,*/ mCargarDefVar, mCargarCiclicidad, mCargarDatos, mSave, /*mImpCasos,*/ mExit, mAcercaDe;

	/** Let choose ciclicidad file when importing eTools */
	JRadioButtonMenuItem activarCiclicidad;
	
	/** Let show variables and cases information on console */
	JRadioButtonMenuItem MostrarInfoConsola;
	/** */
	//JTextField ficheroCiclicidad;
	//TTTJLabel ficheroCiclicidad;
	
	/** Let choose the distance tu use while creating the CBR library */
	JRadioButtonMenuItem distManhattanDiscreta,distEucledianaContinua,distCutremoto,distTanimoto;
	
	/** A GUI's area */
	JPanel pCenter, pCenterLeft, pVar, pCurrCases, pCbr, pMessages;


	//------------------ variables de los datos ----------------------

	/** The Case Library */
	EToolsCBR cbr;
	
	/** The name of the file where information is loaded and/or saved */ 
	
	/** The attribute table */ 
	EToolsAttributeTable atrTab;

	/** Ciclicidad selected flag. True if ciclicidad is selected */
	boolean activarCiclicidadFlag  = prefs.getBoolean("CiclicidadSelect",false);
	
	/** If true shows all kind of messages tu the user. If false shows only messages needed */
	boolean debug = prefs.getBoolean("debug",false);;
		
	/** Let choose the distance to use while creating the CBR library */
	String distancia = prefs.get("distanciaSelect",sDistManhattanDiscreta);
	
	/** Nombres asociados a los menus */
	String sNuevoCBR = "Nueva libreria CBR";									// Menu 1_1
	String sCargarCBR = "Cargar libreria CBR";								// Menu 1_2
	String sCrearCBR = "Crear libreria CBR rapida";							// Menu 1_3
	String sCrearIndices = "Crear Fichero Indices"; 						// Menu 1_X (Anulada)
	String sCrearlibreriaCBR = "Crear libreria CBR paso a paso";		// Menu 1_4
	String sCargarDefVar = "1.- Cargar definicion de variables";		// Menu 1_4_1
	String sCiclicidad = "2.- Activar y cargar ciclicidad (OPCIONAL)";// Menu 1_4_2
	String sCargarCiclicidad = "Cargar fichero de ciclicidad";			// Menu 1_4_2_1
	String sActivarCiclicidad = "Activar ciclicidad";						// Menu 1_4_2_2
	String sDistancia = "3.- Escoger distancia a usar"; 					// Menu 1_4_3
	static String sDistManhattanDiscreta = "Manhattan Discreta"; 		// Menu 1_4_3_1
	static String sDistEucledianaContinua = "Euclediana Continua";		// Menu 1_4_3_2
	static String sDistCutremoto = "Cutremoto";								// Menu 1_4_3_3
	static String sDistTanimoto = "Tanimoto";									// Menu 1_4_3_4
	String sCargarDatos = "4.- Cargar fichero de datos con los casos";// Menu 1_4_4
	String sSalvarCBR = "5.- Guardar libreria CBR";							// Menu 1_4_5
	String sCargarCasos = "Cargar Casos Actuales";							// Menu 1_X (Anulada)
	String sSalir = "Salir...";													// Menu 1_5
	String sMostrarInfoConsola = "Mostrar informacion por consola";	// Menu 2_1
	String sAcercaDe = "Acerca de...";											// Menu 3_1


  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================

  /**
	* Creates a Case Library Builder.<br>
	* {PRE: true
	* }<br>
	* {POST: an instance of CBRControlPanel is created
	* }<br>
	* @param title the title of the application's window.
	*/ 
	public CBRCompiler (String title)
	{
		super (title);
	}	 

  /**
	* Constructor por defecto.<br>
	* {PRE: true
	* }<br>
	* {POST: an instance of CBRControlPanel is created
	* }<br>
	* @param title the title of the application's window.
	*/ 
	public CBRCompiler () {}

	/**
	* Funcion que crea una libreria CBR de forma autonoma sin mostrar el interfaz grafico.<br>
	* {PRE: true
	* }<br>
	* {POST: an instance of CBRControlPanel is created
	* }<br>
	* @param title the title of the application's window.
	*/ 
	public boolean crearLibreriaCBR (String defVarFichero,String datosFichero,String libreriaCBR,String ciclicidadFichero, String dist, boolean debuguear)
	{
		boolean error = false;
		String msg = null;
		if ( !(libreriaCBR.toLowerCase()).endsWith(".cbr"))
			libreriaCBR = libreriaCBR + ".cbr";
		System.out.println("- Fichero de definicion de variables: " + defVarFichero);
		System.out.println("- Fichero de datos: " + datosFichero);
		System.out.println("- Fichero de la libreria CBR: " + libreriaCBR);
		if( !(new File(defVarFichero).exists()) )
		{
			msg = "ERROR: El fichero " + defVarFichero + " no existe.\n";
			error = true;
		}
		if( !(new File(datosFichero).exists()) )
		{
			msg = "ERROR: El fichero " + datosFichero + " no existe.\n";
			error = true;
		}
		if ( ciclicidadFichero != null )
		{
			if( !(new File(ciclicidadFichero).exists()) )
			{
				msg = "ERROR: El fichero " + ciclicidadFichero + " no existe.\n";
				error = true;
			}
			else
				System.out.println("- Fichero de ciclicidad: " + ciclicidadFichero);
		}
		else
			System.out.println("- Ciclicidad de variables desactivada.");;
		/*
		if ( !(new File(libreriaCBR).canWrite()) )
		{
			msg = "ERROR: El fichero " + libreriaCBR + " no se puede escribir.\n";
			error = true;
		}
		*/
		if (dist != null)
		{
			if ((dist.toUpperCase()).compareTo(sDistManhattanDiscreta.toUpperCase()) == 0)
				distancia = sDistManhattanDiscreta;
			else if ((dist.toUpperCase()).compareTo(sDistEucledianaContinua.toUpperCase()) == 0)
				distancia = sDistEucledianaContinua;
			else if ((dist.toUpperCase()).compareTo(sDistCutremoto.toUpperCase()) == 0)
				distancia = sDistCutremoto;
			else if ((dist.toUpperCase()).compareTo(sDistTanimoto.toUpperCase()) == 0)
				distancia = sDistTanimoto;
			else
			{
				msg = "ERROR: No se encuentra la distancia seleccionada.";
				error = true;
			}
			if (!error)
				System.out.println("- Usando distancia: " + distancia);
		}
		else
			System.out.println("- Usando distancia por defecto: " + distancia);
		if (debuguear == true)
		{
			debug = debuguear;
			System.out.println("Activado modo debugueo con informacion extra por consola.");
		}
		System.out.println("");
		if (!error)
		{		
			System.out.println("Cargando fichero de definicion de variables: " + defVarFichero);
			msg = getVars(defVarFichero); // Procesa fichero de definicion de variables
			System.out.println("*****************************\n" + msg + "\n*****************************\n");
			if ((atrTab.toString()).compareTo("{}") == 0)
				System.out.println("ERROR::No se ha leido nada del fichero de definicion de variables");
			else
			{
				if (ciclicidadFichero != null)
					error = cargarCiclicidad(ciclicidadFichero);		// y no se produce error al cargar
				if (!error)
				{	// Si no error al cargar ciclicidad (si cargada)
					System.out.println("Cargando fichero de datos: " + datosFichero);
					msg = getData(datosFichero);				// Procesa fichero de datos
					System.out.println("*****************************\n" + msg + "\n*****************************\n");
					if (cbr.size() == 0)
					{
						System.out.println("ERROR::No se ha leido ningun caso");
						error = true;
					}
					else
					{
						System.out.println("Guardando base de datos CBR: " + libreriaCBR);
						msg = saveCBR(libreriaCBR);
						if (msg.compareTo("ERROR GUARDANDO FICHERO") == 0)
							error = true;
						System.out.println("*****************************\n" + msg + "\n*****************************\n");
					}
				}
			}
		}
		else
			System.out.println("*****************************\n" + msg + "\n*****************************\n");
		return (error);
	}	 

	
  //====||===============================================================
  //===\||/=================== metodo Main ==============================
  //====\/===============================================================


  /** 
	* Creates a Case Library Builder and starts its execution. <br>
	* No parameters are needed during the call of the program.
	* <br>
	* {PRE: true
	* }<br>
	* {POST: the execution of the program has ended
	* }<br>
	* @param args the run-time arguments of the application (that are skipped).
	*/
	public static void main (String [] args)
	{
		boolean debuguear = false;
		int numargs = args.length;
		if (numargs == 0)
		{
			System.out.println("Arrancando modo grafico de la aplicacion.");
			CBRCompiler cpan = new CBRCompiler("E-Tools Case Library Builder v1.3");
			cpan.createGUI();
		}
		else if ( (numargs > 1) )
		{
			boolean error = false;				// true si error durante el proceso de datos
			String msg = null;					// Almacena los mensajes para el usuario
			String defVarFichero = args[0];	// Nombre y ruta del fichero de definicion de variables
			String ciclicidadFichero = null;	// Nombre y ruta del fichero de ciclicidad de variables
			String dist = null;					// Almacena la distancia a usar para la libreria CBR
			String datosFichero = args[1];	// Nombre y ruta del fichero de datos
			String libreriaCBR = null;			// Nombre y ruta del fichero de salida de la libreria CBR
			
			//System.out.println("Arrancando aplicacion en modo consola.");
			if (numargs > 2)
			{
				if (numargs > 3)
				{
					for (int i = 2;i < args.length;i++)
					{
						if (args[i].compareTo("-c") == 0)
						{
							if (args.length > (i + 1) )
							{
								numargs -= 2;
								ciclicidadFichero = args[i+1];
							}
							else
								error = true;
						}
						if (args[i].compareTo("-d") == 0)
						{
							if (args.length > (i + 1) )
							{
								numargs -= 2;
								dist = args[i+1];
							}
							else
								error = true;
						}
						if (args[i].compareTo("-debug")==0)
							debuguear = true;	// Activa informacion extra por consola
						if (error)
							break;
					}
					if (numargs == 3)
						libreriaCBR = args[2];
					else
						libreriaCBR = datosFichero.substring(0,datosFichero.length()-4) + ".cbr";
				}
				else
				{
					if (args[2].compareTo("-debug")==0)
					{
						debuguear = true;
						libreriaCBR = datosFichero.substring(0,datosFichero.length()-4) + ".cbr";
					}
					else
						libreriaCBR = args[2];
				}
			}
			else
				libreriaCBR = datosFichero.substring(0,datosFichero.length()-4) + ".cbr";
			
			if (!error)
			{
				CBRCompiler cpan = new CBRCompiler();
				error = cpan.crearLibreriaCBR(defVarFichero,datosFichero,libreriaCBR,ciclicidadFichero,dist,debuguear);
				cpan.dispose();
				msg = "Errores durante la creacion de la libreria CBR.";
				if (error)
					System.out.println("*****************************\n" + msg + "\n*****************************\n");
					
			}
			else
				System.out.println("*****************************\n" + msg + "\n*****************************\n");
		}
		else
		{
			System.out.println("ERROR: Numero de argumentos incorrecto.");
			System.out.println("java CBRCompiler - Muestra el interfaz grafico");
			System.out.println("java CBRCompiler fichero1 fichero2 [fichero3] [-c fichero4] [-d distancia] [-debug]");
			System.out.println("\t- fichero1:  Fichero de definicion de variables");
			System.out.println("\t- fichero2:  Fichero de datos para libreria CBR");
			System.out.println("\t- fichero3:  Fichero de salida de la libreria CBR");
			System.out.println("\t             (Si no se especifica se usa fichero2.cbr)");
			System.out.println("\t- fichero4:  Fichero de ciclicidad de variables. Cada variable");
			System.out.println("\t-            va en una linea finalizada por un retorno de carro.");
			System.out.println("\t- distancia: Cadena que indica el tipo de distancia a usar:");
			System.out.println("\t-            - \"Manhattan Discreta\".");
			System.out.println("\t-            - \"Euclidea Continua\".");
			System.out.println("\t-            - \"Cutremoto\".");
			System.out.println("\t-            - \"Tanimoto\".");
			System.out.println("\t- debug:     Muestra informacion extra por pantalla para");
			System.out.println("\t-            debuguear los resultados.");
		}
	}

  /**
	* Creates all the GUI components, builds the GUI and starts the event dispatching process. <br>
	* {PRE: x is an instance of CBRControlPanel
	* }<br>
	* {POST: the window and all its components have been created
	* }<br>	 
	*/
	void createGUI()
	{
		getContentPane().setLayout(new BorderLayout());
		
		//1.1 Menu
		mNew = new JMenuItem(sNuevoCBR);
		mNew.addActionListener(this);
		mNew.setToolTipText("Inicializa el estado del programa.");
		mLoad = new JMenuItem(sCargarCBR);
		mLoad.addActionListener(this);
		mLoad.setToolTipText("Carga una libreria CBR.");
		//XXXmLoad.setAccelerator(KeyStroke.getKeyStroke(new Character('f'),java.awt.event.InputEvent.ALT_MASK));
		mCrearCBR = new JMenuItem(sCrearCBR);
		mCrearCBR.addActionListener(this);
		mCrearCBR.setToolTipText("Carga secuencialmente todos loa datos necesarios para crear una libreria segun las opciones seleccionadas.");
		//mIndexFile = new JMenuItem(sCrearIndices);	// BBB
		//mIndexFile.addActionListener(this);// BBB
		//mIndexFile.setToolTipText("Crea los ficheros de indice de un fichero de datos.");
		{
			mCargarDefVar = new JMenuItem(sCargarDefVar);
			mCargarDefVar.addActionListener(this);
			mCargarDefVar.setToolTipText("Carga el fichero de definicion de variables.");
			{
				activarCiclicidad = new JRadioButtonMenuItem(sActivarCiclicidad);
				activarCiclicidad.setToolTipText("Activa la carga del archivo de ciclicidad de variables al crear libreria CBR rapida");
				activarCiclicidad.addActionListener(this);
				activarCiclicidad.setSelected(activarCiclicidadFlag);
				mCargarCiclicidad = new JMenuItem(sCargarCiclicidad);
				mCargarCiclicidad.addActionListener(this);
				mCargarCiclicidad.setToolTipText("Carga el fichero de ciclicidad y la activa. Desactivo hasta que se cargue el fichero de definicion de variables");
				mCargarCiclicidad.setEnabled(false);
				mCiclicidad = new JMenu(sCiclicidad);
				mCiclicidad.setToolTipText("Activa y/o carga el fichero de ciclicidad de variables.");
				mCiclicidad.add(mCargarCiclicidad);
				mCiclicidad.add(activarCiclicidad);
			}
			mCargarDatos = new JMenuItem(sCargarDatos);
			mCargarDatos.addActionListener(this);
			mCargarDatos.setToolTipText("Carga el fichero de datos que contiene los casos que forman la libreria de casos CBR.");
			//ficheroCiclicidad = new JTextField("Ciclicidad no seleccionada");
			//TTTficheroCiclicidad = new JLabel("    Ciclicidad no seleccionada");
			{
				distManhattanDiscreta = new JRadioButtonMenuItem(sDistManhattanDiscreta);
				distManhattanDiscreta.setToolTipText("Selecciona distancia Manhattan discreta.");
				distManhattanDiscreta.addActionListener(this);
				distManhattanDiscreta.setSelected(distancia.compareTo(sDistManhattanDiscreta ) == 0);
				distEucledianaContinua = new JRadioButtonMenuItem(sDistEucledianaContinua);
				distEucledianaContinua.setToolTipText("Selecciona distancia Euclediana continua.");
				distEucledianaContinua.addActionListener(this);
				distEucledianaContinua.setSelected(distancia.compareTo(sDistEucledianaContinua) == 0);
				distCutremoto = new JRadioButtonMenuItem(sDistCutremoto);
				distCutremoto.setToolTipText("Selecciona distancia Cutremoto.");
				distCutremoto.addActionListener(this);
				distCutremoto.setSelected(distancia.compareTo(sDistCutremoto) == 0);
				distTanimoto = new JRadioButtonMenuItem(sDistTanimoto);
				distTanimoto.setToolTipText("Selecciona distancia Tanimoto.");
				distTanimoto.addActionListener(this);
				distTanimoto.setSelected(distancia.compareTo(sDistTanimoto) == 0);
				ButtonGroup butgrp = new ButtonGroup();		// Grupo botones de exclusion mutua
				butgrp.add(distManhattanDiscreta);				// Se añaden todas las distancias
				butgrp.add(distEucledianaContinua); 			// pues solo se podrá tener activa una
				butgrp.add(distCutremoto); 						// a la vez.
				butgrp.add(distTanimoto);							// 
				mDistancia = new JMenu (sDistancia);
				mDistancia.setToolTipText("Selecciona el tipo de distancia que se usara para la libreria de casos CBR.");
				mDistancia.add(distManhattanDiscreta);
				mDistancia.add(distEucledianaContinua);
				mDistancia.add(distCutremoto);
				mDistancia.add(distTanimoto);
				
			}
			mSave = new JMenuItem(sSalvarCBR);
			mSave.addActionListener(this);
			mSave.setToolTipText("Guarda la libreria CBR correspondiente a los datos cargados en los pasos anteriores.");
			mCrearlibreriaCBR = new JMenu(sCrearlibreriaCBR);
			mCrearlibreriaCBR.setToolTipText("Pasos necesarios para crear una libreria de casoso CBR (el paso 2 es opcional).");
			mCrearlibreriaCBR.add(mCargarDefVar);
			//mImport.addSeparator();
			mCrearlibreriaCBR.add(mCiclicidad);
			mCrearlibreriaCBR.add(mDistancia);
			mCrearlibreriaCBR.add(mCargarDatos);
			mCrearlibreriaCBR.add(mSave);
			//TTTmImport.add(ficheroCiclicidad);
			//mImport.addSeparator();
		}
		//mImpCasos = new JMenuItem(sCargarCasos);
		//mImpCasos.addActionListener(this);
		//mImpCasos.setToolTipText("Permite cargar los casos actuales.");
		mExit = new JMenuItem(sSalir);
		mExit.addActionListener(this);
		mExit.setToolTipText("Finaliza el programa y guarda el estado de las opciones seleccionadas.");
		
		mFile = new JMenu("Fichero");
		mFile.add(mNew);
		mFile.addSeparator();
		mFile.add(mLoad);
		mFile.add(mCrearCBR);
		mFile.addSeparator();
		//mFile.add(mIndexFile);	// XXX Desactivada por desconocer su utilidad
		mFile.add(mCrearlibreriaCBR);
		//mFile.add(mImpCasos); 	// XXX Desactivada por desconocer su utilidad
		mFile.addSeparator();
		mFile.add(mExit);
		
		MostrarInfoConsola = new JRadioButtonMenuItem(sMostrarInfoConsola);
		MostrarInfoConsola.addActionListener(this);
		MostrarInfoConsola.setSelected(debug);
		MostrarInfoConsola.setToolTipText("Muestra informacion de las variables y casos cargados por consola.");
		mOpciones = new JMenu("Opciones");
		mOpciones.add(MostrarInfoConsola);
		
		mHelp = new JMenu("Ayuda");
		mAcercaDe = new JMenuItem(sAcercaDe);
		mAcercaDe.addActionListener(this);
		mAcercaDe.setToolTipText("Información sobre la versión y creador del programa.");
		mHelp.add(mAcercaDe);
		
		mBar =  new JMenuBar();		
		mBar.add(mFile);
		mBar.add(mOpciones);
		mBar.add(mHelp);
		
		setJMenuBar(mBar);
		
		//1.2 Panel Central
		pCenter = new JPanel();
		pCenter.setLayout(new GridLayout(1,2));
		
		//1.2.1 Panel Centro izquierda
		pCenterLeft = new JPanel();
		pCenterLeft.setLayout(new GridLayout(2,1));

		
		//1.2.1.1 Panel de texto para las variables
		pVar = new JPanel();
		pVar.setLayout(new BorderLayout());
		lVar = new JLabel("Definición de las variables");
		tVar = new JTextArea();
		JScrollPane tVarPane = new JScrollPane(tVar);
		tVarPane.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		tVarPane.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
		pVar.add("North",lVar);
		pVar.add("Center",tVarPane);
		
		//1.2.1.2 Panel de texto para los casos actuales
		pCurrCases = new JPanel();
		pCurrCases.setLayout(new BorderLayout());
		lCurrCases = new JLabel("Area de Casos Actuales");
		tCurrCases = new JTextArea();
		JScrollPane tCurrCasesPane = new JScrollPane(tCurrCases);
		tCurrCasesPane.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		tCurrCasesPane.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
		pCurrCases.add("North",lCurrCases);
		pCurrCases.add("Center",tCurrCasesPane);

		pCenterLeft.add(pVar);
		pCenterLeft.add(pCurrCases);

		//1.2.2 Panel de texto para la libreria de casos
		pCbr = new JPanel();
		pCbr.setLayout(new BorderLayout());
		lCbr = new JLabel("Libreria de Casos");
		tCbr = new JTextArea();
		JScrollPane tCbrPane = new JScrollPane(tCbr);
		tCbrPane.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		tCbrPane.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
		pCbr.add("North",lCbr);
		pCbr.add("Center",tCbrPane);
		
		//pCenter.add(pCenterLeft);
		pCenter.add(pCenterLeft);
		pCenter.add(pCbr);

		//1.3 Panel de mensajes
		pMessages = new JPanel();
		Box bMessages = Box.createHorizontalBox();
		lMessages = new JLabel("Mensajes:");
		tMessages = new JTextField(60);
		tMessages.setEditable(false);
		//tMessages.setText("Mensajes al usuario.");
		bMessages.add(lMessages);
		bMessages.add(Box.createHorizontalStrut(10));
		bMessages.add(tMessages);
		pMessages.add(bMessages);
		
		addWindowListener(new CloseListener(this));
		
		//2. Montar la ventana
		getContentPane().add("Center",pCenter);
		getContentPane().add("South",pMessages);
		//pack();
		// Se establece tamaño y posición según la ultima vez que se abrió el interfaz
		setSize(prefs.getInt("width",800),prefs.getInt("height",600));
		setLocation(prefs.getInt("Loc_x",10),prefs.getInt("Loc_y",10));
		setVisible(true);
	}
	
	  /** 
		* When push:
		* - Nuevo CBR: Prepares all the application to build a new Case Library.
		* All the information that has not been saved will be lost. <br>
		* - Cargar CBR: Loads a Case Library from a file. <br>
		* The name of the file is get through a file browser window.
		* - Salvar CBR: Saves a Case Library to a file. <br>
		* The name of the file is get through a file browser window.
		* - Crear Fichero de Indices: Creates an index file from the source data file. <br>
		* The name of the source file is get through a file browser window.
		* - Import e-Tools: Imports the off-line information needed to build a Case Library. <br>
		* The names of the files are get through a file browser window.
		* - Cargar Casos actuales: Loads cases from a file and makes a query to the
		* Case Library with each case loaded. <br>
		* The name of the file is get through a file browser window.
		* Salir: Closes the application.<br>
		* Ayuda/Acerca de:Opens a pop-up window with the "about" information. <br>
		*
		*/
		public void actionPerformed(ActionEvent event)
		{
			//System.out.println("fuente evento: " + event.getSource());
			JMenuItem source = (JMenuItem)(event.getSource());
			
			if ((source.getText()).compareTo(sNuevoCBR) == 0)
			{	//borrar datos antiguos
				atrTab = null;
				cbr = null;
				tVar.setText("");   
				tCbr.setText("");   
				tMessages.setText("");
			}
			else if ((source.getText()).compareTo(sCargarCBR) == 0)
			{
				//cargar fichero CBR
				String lastPath = prefs.get("lastpathCargaCBR","");
				JFileChooser dialog = new JFileChooser(lastPath);
				dialog.setDialogTitle("Cargar CBR de...");	 // Titulo
				dialog.setApproveButtonText("Cargar CBR");
				if(dialog.showOpenDialog(this)==JFileChooser.APPROVE_OPTION) // Muestra dialogo
				{	// Si en el dialogo se escogió aceptar
					prefs.put("lastpathCargaCBR",dialog.getCurrentDirectory().getPath());
					String filename = dialog.getCurrentDirectory().getPath() +"/"+ dialog.getSelectedFile().getName(); 	// Ruta y nombre del fichero seleccionado
					if(new File(filename).exists())
					{
						System.out.println("Cargando fichero: " + filename + "\n");
						String msg = loadCBR(filename);
						tMessages.setText(msg);
						System.out.println("*****************************\n" + msg + "\n*****************************\n");
						tVar.setText(atrTab.toString());
						tCbr.setText(cbr.toString());   
					}
					else
					{
						String msg ="El fichero: " + filename + " no existe";
						System.out.println("*****************************\n" + msg + "\n*****************************\n");
						tMessages.setText(msg);
					}
				}
			}
			else if ((source.getText()).compareTo(sCrearCBR) == 0)
			{	// Modo rápido de crear una libreria CBR. Carga todos los interfaces
				// necesarios secuencialmente mientras no haya error.
				if ( !cargarDefVar() )
				{	// Si no hay error al cargar el fichero de definición de variables
					boolean error = false;
					if (activarCiclicidad.isSelected())	// Si escogido cargar ciclicidad
						error = cargarCiclicidad(null);		// y no se produce error al cargar
					if (!error)
					{	// Si no error al cargar ciclicidad (si cargada)
						if ( !cargarDatos() )	// Si no error al cargar datos
							salvarCBR();			// Salvar la libreria CBR
					}
				}
			}
			/*else if ((source.getText()).compareTo(sCrearIndices) == 0)
			{
				String indexFile = null;
				String lastPath = prefs.get("lastpathIndex","");
				JFileChooser dialog = new JFileChooser(lastPath);
				dialog.setDialogTitle("Abrir fichero de datos de...");	 // Titulo
				dialog.setApproveButtonText("Abrir fichero de dadtos");
				if(dialog.showOpenDialog(this)==JFileChooser.APPROVE_OPTION) // Muestra dialogo
				{	// Si en el dialogo se escogió aceptar
					prefs.put("lastpathIndex",dialog.getCurrentDirectory().getPath());
					String filename = dialog.getCurrentDirectory().getPath() +"/"+ dialog.getSelectedFile().getName(); 	// Ruta y nombre del fichero seleccionado
					if(new File(filename).exists())
					{
						//se crea un fichero de indices con nombre "Index-Nombre_del_arhivo_original"
						String msg = new String("Creating index file...");
						tMessages.setText(msg);
						System.out.println(msg);
						indexFile = createIndexFile( dialog.getCurrentDirectory().getPath() +"/Index-"+dialog.getSelectedFile().getName(), filename);
						if (indexFile!=null)
						{
							msg = "Index File [" + indexFile + "] created.";
							tMessages.setText(msg);
							System.out.println(msg);
						}
						else
						{
							msg = "Index File [" + indexFile + "] not created.";
							tMessages.setText(msg);
							System.out.println(msg);
						}
					}
					else
					{
						String msg ="El fichero: " + filename + " no existe";
						System.out.println("*****************************\n" + msg + "\n*****************************\n");
						tMessages.setText(msg);
					}
				}
			}*/
			else if ((source.getText()).compareTo(sCargarDefVar) == 0)
			{	// Importa los archivos con los datos para crear la librería CBR
				mCargarCiclicidad.setEnabled(false);
				if ( !cargarDefVar() )
					mCargarCiclicidad.setEnabled(true);
			}
			else if ((source.getText()).compareTo(sActivarCiclicidad) == 0)
			{
				activarCiclicidadFlag = activarCiclicidad.isSelected();
				String msg = "";
				if (activarCiclicidad.isSelected())
					msg = "Al importar los datos se pedira el archivo de ciclicidad";
				else
					msg = "Ninguna variable sera activada como ciclica";
				System.out.println("*****************************\n" + msg + "\n*****************************\n");
				tMessages.setText(msg);
			}
			else if ((source.getText()).compareTo(sCargarCiclicidad) == 0)
			{
				activarCiclicidad.setSelected(true);
				activarCiclicidadFlag = activarCiclicidad.isSelected();
				cargarCiclicidad(null);
			}
			else if ((source.getText()).compareTo(sDistManhattanDiscreta) == 0)
			{
				distancia = sDistManhattanDiscreta;
				String msg = "Seleccionada distancia " + distancia + " para la creacion de la libreria CBR";
				System.out.println("*****************************\n" + msg + "\n*****************************\n");
				tMessages.setText(msg);
			}
			else if ((source.getText()).compareTo(sDistEucledianaContinua) == 0)
			{
				distancia = sDistEucledianaContinua;
				String msg = "Seleccionada distancia " + distancia + " para la creacion de la libreria CBR";
				System.out.println("*****************************\n" + msg + "\n*****************************\n");
				tMessages.setText(msg);
			}
			else if ((source.getText()).compareTo(sDistCutremoto) == 0)
			{
				distancia = sDistCutremoto;
				String msg = "Seleccionada distancia " + distancia + " para la creacion de la libreria CBR";
				System.out.println("*****************************\n" + msg + "\n*****************************\n");
				tMessages.setText(msg);
			}
			else if ((source.getText()).compareTo(sDistTanimoto) == 0)
			{
				distancia = sDistTanimoto;
				String msg = "Seleccionada distancia " + distancia + " para la creacion de la libreria CBR";
				System.out.println("*****************************\n" + msg + "\n*****************************\n");
				tMessages.setText(msg);
			}
			else if ((source.getText()).compareTo(sCargarDatos) == 0)
			{	// Importa los archivos con los datos para crear la librería CBR
				cargarDatos();
			}
			else if ((source.getText()).compareTo(sSalvarCBR) == 0)
			{	//guardar CBR en disco
				salvarCBR();
			}
			/*else if ((source.getText()).compareTo(sCargarCasos) == 0)
			{
				//cargar Casos actuales desde fichero
				if (atrTab == null)
				{
					String msg = "No hay ningún fichero cargado";
					tMessages.setText(msg);
					System.out.println(msg);
				}
				else
				{
					String lastPath = prefs.get("lastpathCasos","");
					JFileChooser dialog = new JFileChooser(lastPath);
					dialog.setDialogTitle("Abrir fichero de casos actuales...");	 // Titulo
					dialog.setApproveButtonText("Abrir fichero casos");
					if(dialog.showOpenDialog(this)==JFileChooser.APPROVE_OPTION) // Muestra dialogo
					{	// Si en el dialogo se escogió aceptar
						prefs.put("lastpathCasos",dialog.getCurrentDirectory().getPath());
						String filename = dialog.getCurrentDirectory().getPath() +"/"+ dialog.getSelectedFile().getName(); 	// Ruta y nombre del fichero seleccionado
						if(new File(filename).exists())
						{
							loadCurrCases(filename);
						}
						else
						{
							String msg ="El fichero: " + filename + " no existe";
							System.out.println("*****************************\n" + msg + "\n*****************************\n");
							tMessages.setText(msg);
						}
					}
				}
			}*/
			else if ((source.getText()).compareTo(sMostrarInfoConsola) == 0)
			{
				debug = MostrarInfoConsola.isSelected();
				if (debug)
					System.out.println("Habilitado mostrar informacion de las variables y casos cbr por consola al cargarse los ficheros.");
				else
					System.out.println("Deshabilitado mostrar informacion de las variables y casos cbr por consola al cargarse los ficheros.");
			}
			else if ((source.getText()).compareTo(sSalir) == 0)
			{
				prefs.putInt("width",getWidth());
				prefs.putInt("height",getHeight());
				prefs.putInt("Loc_x",getLocation().x);
				prefs.putInt("Loc_y",getLocation().y);
				prefs.putBoolean("CiclicidadSelect",activarCiclicidadFlag);
				prefs.put("distanciaSelect",distancia);
				prefs.putBoolean("debug",debug);
				System.exit(0);
			}
			else if ((source.getText()).compareTo(sAcercaDe) == 0)
			{
				DialogAbout dialog = new DialogAbout(CBRCompiler.this,"Acerca de...");
				dialog.setVisible(true);
			}
			else
				System.out.println("ERROR: Recibido evento desconocido.");
		}
		
 
	  //====||============================================================
	  //===\||/================ metodos auxiliares =======================
	  //====\/============================================================

	  /** 
		* Loads variables definition file in order to create a new CBR Library.<br>
		* <br>
		* {PRE: y is instance of LoadListener.
		* }<br>
		* {POST: y loads a new Case Library.
		* }<br>
		* @return boolean. true if error.
		*/
		private boolean cargarDefVar()
		{	//cargar fichero variables
			String filename = null;
			boolean error = false;
			String msg = null;
			System.out.println("Modo Grafico activo.");
			String lastPath = prefs.get("lastpathVariables","");
			JFileChooser dialog = new JFileChooser(lastPath);
			dialog.setDialogTitle("Abrir fichero de definición de variables");	 // Titulo
			dialog.setApproveButtonText("Abrir fichero def vars");
			if(dialog.showOpenDialog(this)==JFileChooser.APPROVE_OPTION) // Muestra dialogo
			{	// Si en el dialogo se escogió aceptar
				prefs.put("lastpathVariables",dialog.getCurrentDirectory().getPath());
				filename = dialog.getCurrentDirectory().getPath() +"/"+ dialog.getSelectedFile().getName(); 	// Ruta y nombre del fichero seleccionado
				if(new File(filename).exists())
				{
					msg = getVars(filename);	// Procesa fichero de definicion de variables
					tMessages.setText(msg);
					System.out.println("*****************************\n" + msg + "\n*****************************\n");
					tVar.setText(atrTab.toString());   
					if ((atrTab.toString()).compareTo("{}") == 0)
					{
						msg = "ERROR::No se ha leido nada del fichero de definicion de variables";
						tMessages.setText(msg);
						System.out.println("*****************************\n" + msg + "\n*****************************\n");
						error = true;
					}
				}
				else
				{
					msg ="El fichero: " + filename + " no existe";
					System.out.println("*****************************\n" + msg + "\n*****************************\n");
					tMessages.setText(msg);
					error = true;
				}
			}
			else
			{
				msg = "Carga de archivo anulada";
				System.out.println("*****************************\n" + msg + "\n*****************************\n");
				tMessages.setText(msg);
				error = true;
			}
			return (error);
		}

		/**
		* Carga el archivo conteniendo las variables que han de ser establecidas como ciclicicases.<br>
		* <br>
		* {PRE: y is instance of ImportListener.
		* }<br>
		* {POST: y loads the parameter descriptors and creates an attribute table.
		* }<br>
		* @return String a message telling the success/failure of the loading process.
		*/
		private boolean cargarCiclicidad(String ciclicidadFile)
		{	// Comprueba si está seleccionada la lectura del archivo conteniendo
			// las variables que deben ser cíclicas
			AttributeDef auxDef;
			String msg = null;
			String filename = null;
			boolean error = false;
			if (ciclicidadFile == null)
			{
				String lastPath = prefs.get("lastpathCiclicidad","");
				JFileChooser dialog = new JFileChooser(lastPath);
				dialog.setDialogTitle("Abrir fichero de ciclicidad de variables");	 // Titulo
				dialog.setApproveButtonText("Abrir fichero de ciclicidad");
				if(dialog.showOpenDialog(this)==JFileChooser.APPROVE_OPTION) // Muestra dialogo
				{	// Si en el dialogo se escogió establecer ciclicidad de variables
					prefs.put("lastpathCiclicidad",dialog.getCurrentDirectory().getPath());
					filename = dialog.getCurrentDirectory().getPath() +"/"+ dialog.getSelectedFile().getName();		// Ruta y nombre del fichero seleccionado
				}
				else
				{
					error = true;
					System.out.println("No se ha seleccionado ningún fichero");
					tMessages.setText("No se ha seleccionado ningún fichero");
				}
			}
			else
				filename = ciclicidadFile;
			if(new File(filename).exists())
			{
				System.out.println("Cargando fichero de ciclicidad: " + filename + "\n");
				try
				{
					BufferedReader in=new BufferedReader(
						new FileReader(filename)); // Lector de Fitxer
					String variable;
					msg = "Estableciendo cíclicidad de las variables:\n";
					String line = "";
					boolean end_of_file = false;
					int i = 0;
					while (!end_of_file)
					{
						line = in.readLine();
						//System.out.println(line);
						if ( (line == null) || (line.compareTo("")==0) )
						{	
							end_of_file = true;
							in.close();
						}
						else
						{	  
							i++;
							line = line.trim();
							//System.out.println("Elemento[" + i + "]: " + line);
							//System.out.println("*************" + atrTab.toString());
							auxDef = (AttributeDef) atrTab.get(line);
							if (auxDef == null)
							{
								msg += line + "\t- ERROR: Etiqueta no encontrada\n";
								error = true;
							}
							else
							{
								msg += line + "\t- Establecida como ciclica\n";
								auxDef.setCyclic(true);  
							}
							//System.out.println(auxDef);
						}
					}
					if (msg.compareTo("Estableciendo cíclicidad de las variables:\n") == 0)
					{
						error = true;
						msg += "ERROR: Fichero vacio.";
					}
					System.out.println(msg);
					if (ciclicidadFile == null)
						tCurrCases.setText(msg);
				}
				catch(Exception e){ e.printStackTrace(System.out); error = true; } 
			}
			else
			{
				error = true;
				System.out.println("*****************************\n" + "El fichero: " + filename + " no existe"+ "\n*****************************\n");
			}
			if (error)
				msg = "ERROR: Problemas estableciendo la ciclicidad de variables.";
			else
				msg = "Fichero de ciclicidad de variables cargado correctamente.";
			System.out.println("*****************************\n" + msg + "\n*****************************\n");
			if (ciclicidadFile == null)
				tMessages.setText(msg);
			return error;
		}
		
	  /** 
		* Loads data file in order to create a new CBR Library.<br>
		* <br>
		* {PRE: y is instance of LoadListener.
		* }<br>
		* {POST: y loads a new Case Library.
		* }<br>
		* @return boolean. true if error.
		*/
		private boolean cargarDatos()
		{	//cargar fichero datos
			String filename = null;
			boolean error = false;
			String msg = null;			
			System.out.println("Modo Grafico activo.");
			String lastPath = prefs.get("lastpathDatos","");
			JFileChooser dialog = new JFileChooser(lastPath);
			dialog.setDialogTitle("Abrir fichero de datos");	 // Titulo
			dialog.setApproveButtonText("Abrir fichero datos");
			if(dialog.showOpenDialog(this)==JFileChooser.APPROVE_OPTION) // Muestra dialogo
			{	// Si en el dialogo se escogió aceptar
				prefs.put("lastpathDatos",dialog.getCurrentDirectory().getPath());
				filename = dialog.getCurrentDirectory().getPath() +"/"+ dialog.getSelectedFile().getName();		// Ruta y nombre del fichero seleccionado
				if(new File(filename).exists())
				{
					msg = getData(filename);		// Procesa fichero de datos
					//se introduce la informacion de los casos a partir del fichero de indices
					tMessages.setText(msg);
					System.out.println("*****************************\n" + msg + "\n*****************************\n");
					tCbr.setText(cbr.toString());   
					if ( ((cbr.toString()).trim()).startsWith("CBR: numero de casos = 0"))
					{
						msg = "ERROR::No se ha leido ningun caso";
						tMessages.setText(msg);
						System.out.println("*****************************\n" + msg + "\n*****************************\n");
						error = true;
					}
				}
				else
				{
					System.out.println("*****************************\n" + "El fichero: " + filename + " no existe"+ "\n*****************************\n");
					error = true;
				}
			}
			else
			{
				msg = "Carga de archivo anulada";
				System.out.println("*****************************\n" + msg + "\n*****************************\n");
				tMessages.setText(msg);
				error = true;
			}
			return (error);
		}
		
		/** 
		* Loads files (variables definition and data) in order to create a new CBR Library.<br>
		* <br>
		* {PRE: y is instance of LoadListener.
		* }<br>
		* {POST: y loads a new Case Library.
		* }<br>
		* @return boolean. true if error.
		*/
		private boolean salvarCBR()
		{
			//guardar CBR en disco
			boolean error = false;
			System.out.println("Modo grafico activo.");
			String lastPath = prefs.get("lastpathGuardaCBR","");
			JFileChooser dialog = new JFileChooser(lastPath);
			dialog.setDialogTitle("Guardar CBR en...");	 // Titulo
			dialog.setApproveButtonText("Salvar CBR");
			if(dialog.showOpenDialog(this)==JFileChooser.APPROVE_OPTION) // Muestra dialogo
			{	// Si en el dialogo se escogió aceptar
				prefs.put("lastpathGuardaCBR",dialog.getCurrentDirectory().getPath());
				String filename = dialog.getCurrentDirectory().getPath() +"/"+ dialog.getSelectedFile().getName(); 	// Ruta y nombre del fichero seleccionado
				System.out.println("Guardando base de datos CBR: " + filename);
				String msg = saveCBR(filename);
				if (msg.compareTo("ERROR GUARDANDO FICHERO") == 0)
					error = true;
				tMessages.setText(msg);   
				System.out.println("*****************************\n" + msg + "\n*****************************\n");
			}
			else
			{
				System.out.println("No se ha seleccionado ningún fichero");
				tMessages.setText("No se ha seleccionado ningún fichero");
				error = true;
			}
			return (error);
		}
		
	  /** 
		* Loads a Case Library from a file using the Java serialization
		* mechanism. <br>
		* <br>
		* {PRE: y is instance of LoadListener.
		* }<br>
		* {POST: y loads a new Case Library.
		* }<br>
		* @return String a message telling the success/failure of the loading process.
		*/
		private String loadCBR(String filename)
		{
			//Cargar el CBR de disco
			try
			{
				//Obrim el fitxer per llegir la llibreria.
				ObjectInputStream fichero = new ObjectInputStream(
						  new FileInputStream(filename));

				cbr = (EToolsCBR)fichero.readObject(); 	 
			}
			catch (IOException e)
			{
				e.printStackTrace(System.out);
			}
			catch (ClassNotFoundException e2)
			{
				e2.printStackTrace(System.out);
			}
	 
			atrTab = (EToolsAttributeTable) cbr.getAttributeTable();
		  
			return "El CBR se ha cargado con éxito";
		}
		
	  /** 
		* Saves a Case Library to a file using the Java serialization
		* mechanism. <br>
		* <br>
		* {PRE: y is instance of SaveListener.
		* }<br>
		* {POST: y saves a new Case Library.
		* }<br>
		* @return String a message telling the success/failure of the saving process.
		*/
		private String saveCBR(String filename)
		{	//Grabar el CBR en disco
			String msg = "CBR guardado con éxito";
			ObjectOutputStream fichero = null;
			try
			{
				fichero = new ObjectOutputStream(new FileOutputStream(filename));
				fichero.writeObject(cbr);
				fichero.close();
			}
			catch (Exception ex)
			{
				ex.printStackTrace(System.out);
				msg = "ERROR GUARDANDO FICHERO";
				try { if (fichero!=null) fichero.close(); }
				catch (Exception e) { e.printStackTrace (System.out); }
			}
			return msg;
		}

	  /** 
		* Create Index File from a CBR library.<br>
		* <br>
		* {PRE: y is instance of SaveListener.
		* }<br>
		* {POST: y saves a new Case Library.
		* }<br>
		* @return String a message telling the success/failure of the saving process.
		*/
		private String createIndexFile(String indexFile, String filename)
		{
			String txt = "";
			RandomAccessFile in=null;
			PrintWriter out = null;
			PrintWriter out_gr = null;
			boolean end_of_file = false;
			String line = null;
			int case_number = 0;
			int line_number = 0;
			boolean in_body = false;
	
			//variables para estimacion de tiempo para acabar
			long bytes_leidos = 0;
			long bytes_totales = 0;
			long milisec_at_init = 0;
			long milisec_from_init = 0;
			long milisec_to_end = 0;
			float percent_done = (float) 0.0;
	  
			try
			{
				//System.out.println("voy a leer el fichero");
				in=new RandomAccessFile(filename, "r"); // Lector de Fitxer
				bytes_totales = in.length();
				milisec_at_init = System.currentTimeMillis();
				out=new PrintWriter(new FileWriter(indexFile));
				System.out.println("empieza la creación del fichero resumen. Origen = " + bytes_totales + " bytes."); 		  
				String msg;
				while (!end_of_file)
				{
					line = in.readLine();
					line_number++;
					if (line == null)
					{
						end_of_file = true;
					}
					else if ((line.startsWith("CASE")) && in_body)
					{
						case_number++;
						out.println(line);
						bytes_leidos = in.getFilePointer();
						milisec_from_init = System.currentTimeMillis() - milisec_at_init;
						percent_done = ( ((float) bytes_leidos) / ((float) bytes_totales)) * ((float) 100.0);
						milisec_to_end = (long) ( 
								( ( ((float) bytes_totales) / ((float) bytes_leidos) )
									- ((float) 1.0)
								) * ((float) milisec_from_init) 
							); 				
						msg = "Creating index file... CASE " + case_number +
							 " (" + percent_done +"%, elapsed time = " + 
							(milisec_to_end / 1000) + " seconds)";
						tMessages.setText(msg);
						System.out.println(msg+"\n");
					}
					else if ((line.startsWith("OUTPUTP")) && in_body)
					{
						out.println(line);
						out.println("STARTS_IN_LINE="+line_number);
						out.println("STARTS_IN_BYTE="+Long.toString(in.getFilePointer()));
			 
						out_gr =  new PrintWriter(new FileWriter("../CASE_"+case_number+".dat"));
				
						while ((!end_of_file) && (!line.startsWith("END")))
						{
							line = in.readLine();
							out_gr.println(line);
							out_gr.flush();
							line_number++;
							if (line == null)
							{
								end_of_file = true;
							}
							else if (line.startsWith("END"))
							{
								out.println("ENDS_IN_LINE="+line_number);
								out.println("ENDS_IN_BYTE="+Long.toString(in.getFilePointer()));
								out.println(line);
							}
							//else pasamos de ella
						}
						out_gr.close();
					}
					else if ((line.startsWith("OUTPUTG"))
						||(line.startsWith("SOLUTION"))|| (line.startsWith("MOREINFO")))
					{
						out.println(line);
						out.println("STARTS_IN_LINE="+line_number);
						out.println("STARTS_IN_BYTE="+Long.toString(in.getFilePointer()));
			
						while ((!end_of_file) && (!line.startsWith("END")))
						{
							line = in.readLine();
							line_number++;
							if (line == null)
							{
								end_of_file = true;
							}
							else if (line.startsWith("END"))
							{
								out.println("ENDS_IN_LINE="+line_number);
								out.println("ENDS_IN_BYTE="+Long.toString(in.getFilePointer()));
								out.println(line);
							}
							//else pasamos de ella
						}
					}
					else if (line.startsWith("BODY"))
					{
						in_body = true;	
					}
					else
					{
						out.println(line);
					}
				}
				out.flush();
				msg = "Index file... done";
				tMessages.setText(msg);
				System.out.println(msg+"\n");
				out.close();
			} 
			catch (FileNotFoundException e)
			{
				e.printStackTrace(System.out);
				try
				{
					if (in!=null) in.close();
					if (out!=null) out.close();
					indexFile = null;
				}
			 catch (IOException e2) {e2.printStackTrace(System.out);}
			} 
			catch (IOException e) 
			{
				e.printStackTrace(System.out);
				indexFile = null;
			}
			return indexFile;
	  }
			

	  /**
		* Loads the RTXPS parameter descriptors from a file, creating an AttributeTable with it. <br>
		* <br>
		* {PRE: y is instance of ImportListener.
		* }<br>
		* {POST: y loads the parameter descriptors and creates an attribute table.
		* }<br>
		* @return String a message telling the success/failure of the loading process.
		*/
		private String getVars(String filename)
		{
			String txt = "";
			BufferedReader in=null;
			try
			{
				//System.out.println("voy a leer el fichero");
				in=new BufferedReader(
						 new FileReader(filename)); // Lector de Fitxer
				//System.out.println("voy a parsearlo"); 		  
				atrTab = EToolsAttributeTable.importFromRTXPS(in,debug);
			}
			catch (FileNotFoundException e)
			{
				e.printStackTrace(System.out);
				try{ if (in!=null) in.close(); }
				catch (IOException e2) {e2.printStackTrace(System.out);}
			} 
			catch (IOException e) 
			{
				e.printStackTrace(System.out);
			}	 
			return "fin de la carga de variables";
		}

				
	  /** 
		* Loads the Simulation Data from a file, creating a case for each element of the data. <br>
		* <br>
		* {PRE: y is instance of ImportListener.
		* }<br>
		* {POST: y loads the simulation data and fills the Case Library with it.
		* }<br>
		* @return String a message telling the success/failure of the loading process.
		*/
		private String getData(String filename)
		{
			String txt = "";
			BufferedReader in=null;

			//System.out.println("");
			//System.out.println("########################################################");
			//System.out.println("########################################################");
			//System.out.println("########################################################");
			//System.out.println("########################################################");
			//System.out.println("########################################################");
			//System.out.println("");
	
			try
			{
				//System.out.println("voy a leer el fichero");
				in=new BufferedReader(new FileReader(filename)); // Lector de Fitxer
				//System.out.println("voy a parsearlo");
				cbr = new EToolsCBR();
				cbr.setAttributeTable(atrTab);
				ComposedDistance dist = null;
	
				System.out.println("Usando distancia " + distancia + " para la creacion de la libreria CBR");
				if (distancia.compareTo(sDistManhattanDiscreta) == 0)
					dist = new DiscreteManhattan();
				else if (distancia.compareTo(sDistEucledianaContinua) == 0)
					System.out.println("Usando distancia " + distancia + " para la creacion de la libreria CBR");
				else if (distancia.compareTo(sDistCutremoto) == 0)
					System.out.println("Usando distancia " + distancia + " para la creacion de la libreria CBR");
				else if (distancia.compareTo(sDistTanimoto) == 0)
					dist = new TanimotoDistance();

				cbr.setDistanceMeasure(dist);
				cbr.importFromData(in,debug);
			}
			catch (FileNotFoundException e)
			{
				e.printStackTrace(System.out);
				try{ if (in!=null) in.close(); }
				catch (IOException e2) {e2.printStackTrace(System.out);}
			}
			catch (IOException e)
			{
				e.printStackTrace(System.out);
			}
			return "Fin de la carga de datos.";
		}


	  /** 
		* Loads cases from a file, starting a reasoning cycle with each one.<br>
		* <br>
		* {PRE: y is instance of LoadListener.
		* }<br>
		* {POST: y loads the cases, stars a reasoning cycle for each one and
		*			returns the results through the interface.
		* }<br>
		*/
		public void loadCurrCases(String filename)
		{
			String txt = "";
			BufferedReader in=null;
			java.util.Vector casosActuales;
			EToolsCase casAux;
			RetrievedCase casRetr;
			
			try
			{
				//System.out.println("voy a leer el fichero");
				in=new BufferedReader(
						new FileReader(filename)); // Lector de Fitxer
				//System.out.println("voy a parsearlo"); 		  
				casosActuales = EToolsCBR.loadCurrentCases(in,
										atrTab,
										cbr.getDistanceMeasure());
		
				//recorrido del vector casosActuales 
				//tCurrCases.setText("numero de casos actuales leidos: " + casosActuales.size() +
				// 		"\n" +  casosActuales.toString());		  
				tCurrCases.setText("");
				for (java.util.Iterator i = casosActuales.iterator(); i.hasNext(); )
				{
					casAux = (EToolsCase)i.next();
					cbr.setCurrentCase(casAux);
					casRetr = cbr.retrieve1Case();
					tCurrCases.append("\n caso actual: " + casAux +
						"caso más similar: " + casRetr + 
						"\n info completa\n" + 
						casRetr.getCase().toString());
				}
				
				//tMessages.setText(getVars());
				//tVar.setText(atrTab.toString());		
			} 
		
			catch (FileNotFoundException e)
			{
				System.out.println("ERROR::Fichero actual inexistente.");
				e.printStackTrace(System.out);
				try
				{
					if (in!=null) in.close();
				} 
				catch (IOException e2) {e2.printStackTrace(System.out);}
			}
			catch (IOException e) 
			{
				e.printStackTrace(System.out);
			}
		}
		 

  //====||===============================================================
  //====||===============================================================
  //====||===============================================================
  //===\||/================= Clases Internas ============================
  //====\/===============================================================

  /**
	* This inner class dispatches all the window events of the Case Library Builder's GUI (eg: minimize, close).
	* @see CBRControlPanel
	*/
	private class CloseListener extends WindowAdapter
	{

	  //====||============================================================
	  //===\||/=============== metodos principales =======================
	  //====\/============================================================

		JFrame GUI;
		
		CloseListener(JFrame auxGUI)
		{
			GUI = auxGUI;
		}
	  
	  /** 
		* Closes the application.<br>
		* <br>
		* {PRE: x is instance of CBRControlPanel,
		*		  y is instance of WinListener and
		*		  y receives a command to close the window of x.
		* }<br>
		* {POST: y makes x to stop its execution
		* }<br>
		* @param event the event triggering this method.
		*/
		public void windowClosing(WindowEvent event)
		{
			prefs.putInt("width",GUI.getWidth());
			prefs.putInt("height",GUI.getHeight());
			prefs.putInt("Loc_x",GUI.getLocation().x);
			prefs.putInt("Loc_y",GUI.getLocation().y);
			prefs.putBoolean("CiclicidadSelect",activarCiclicidadFlag);
			prefs.put("distanciaSelect",distancia);
			prefs.putBoolean("debug",debug);
			GUI.dispose();
			System.exit(0);
		}
	}
	
  /**
	* This inner class implements the pop-up window to display the "About" information of the application.<br>
	* <br>
	* @see CBRControlPanel
	* @see AboutListener
	*/
	private class DialogAbout extends Dialog implements ActionListener 
	{
		private final static long serialVersionUID = 100;
		//====||============================================================
		//===\||/=================== atributos =============================
		//====\/============================================================

		/**Button to close the pop-up window*/
		private JButton okButton;

		/**Label of the "About" pop-up window*/
		private JLabel label1, label2;

		/**Label's area*/
		private JPanel pLabels;


	  //====||============================================================
	  //===\||/=============== metodos principales =======================
	  //====\/============================================================

	  /**
		* Creates an DialogAbout with the given parent window and the given title.<br>
		* It opens a pop-up window with some information of the program. 
		* <br>
		* {PRE: fr is a Java Frame (the parent window),
		*		  title is string suitable (informative) for naming the pop-up window 
		* }<br>
		* {POST: an instance of DialogAbout has been created and displayed on the screen.
		* }<br>
		* <br>
		* @param fr the parent Frame.
		* @param title the title of the pop-up window. 
		*/
		public DialogAbout(JFrame fr, String title) 
		{
			super(fr, title, true);
			setLayout(new BorderLayout());
			setBounds(300,300,200,100);

			setBackground(Color.lightGray);

			pLabels = new JPanel();
			pLabels.setLayout(new GridLayout(2,1));
			label1 = new JLabel("CBR Control Panel v2.3");
			label1.setFont(new Font("Courier",Font.BOLD,18));
			//label1.setBackground(Color.green);
			label2 = new JLabel("KEMLg Group 2000-2002");
			pLabels.add(label1);
			pLabels.add(label2);

			add("Center",pLabels);

			okButton = new JButton("ok");
			okButton.addActionListener(this);
			okButton.setSize(50,40);
			add("South",okButton);
	
			/*
			addWindowListener(new WindowAdapter() 
				{
					public void windowClosing(WindowEvent event) 
					{
						setVisible(false);
					}
				}
			);
			*/
			pack();
		}
		
		/** 
		 * Makes the pop-up menu to dissapear. <br>
		 * Instead of destroying the menu, it is make invisible, to be 
		 * used again next time it is requested.
		 * <br>
		 * {PRE: x is instance of DialogAbout,
		 *  y is instance of OkListener and
		 *  y receives a command to close x.
		 * }<br>
		 * {POST: y makes x dissapear.
		 * }<br>
		 * @param e the event triggering this method.
		 */
		 public void actionPerformed(ActionEvent e)
		 {
			 setVisible(false);
		 }
		
	}
}
