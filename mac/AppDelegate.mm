#import "AppDelegate.h"
#import "MainWindowController.h"

@interface AppDelegate ()
@property (nonatomic, strong) MainWindowController* mainWindowController;
@end

@implementation AppDelegate

// Menu-bar layout follows the macOS HIG ("The menu bar"): App, File, Edit,
// View, app-specific menus, Window, Help. Every toolbar command is also a
// menu command (the toolbar can be hidden), and unavailable items are
// disabled -- never hidden -- via MainWindowController's -validateMenuItem:.

// An item handled by the window controller (targeted explicitly: the
// controller exists before the menu is built, see
// -applicationDidFinishLaunching:).
- (NSMenuItem*)addItemTo:(NSMenu*)menu title:(NSString*)title action:(SEL)action key:(NSString*)key
                modifiers:(NSEventModifierFlags)modifiers tag:(NSInteger)tag {
    NSMenuItem* item = [menu addItemWithTitle:title action:action keyEquivalent:key];
    item.keyEquivalentModifierMask = modifiers;
    item.target = self.mainWindowController;
    item.tag = tag;
    return item;
}

- (NSMenuItem*)addItemTo:(NSMenu*)menu title:(NSString*)title action:(SEL)action key:(NSString*)key {
    return [self addItemTo:menu title:title action:action key:key modifiers:NSEventModifierFlagCommand tag:0];
}

// A standard item that goes through the responder chain (no explicit target).
- (NSMenuItem*)addStandardItemTo:(NSMenu*)menu title:(NSString*)title action:(SEL)action key:(NSString*)key
                        modifiers:(NSEventModifierFlags)modifiers {
    NSMenuItem* item = [menu addItemWithTitle:title action:action keyEquivalent:key];
    item.keyEquivalentModifierMask = modifiers;
    return item;
}

- (NSMenu*)addTopLevelMenuTitled:(NSString*)title to:(NSMenu*)mainMenu {
    NSMenuItem* menuItem = [[NSMenuItem alloc] init];
    [mainMenu addItem:menuItem];
    NSMenu* menu = [[NSMenu alloc] initWithTitle:title];
    menuItem.submenu = menu;
    return menu;
}

- (void)buildMainMenu {
    const NSEventModifierFlags cmd = NSEventModifierFlagCommand;
    const NSEventModifierFlags cmdShift = NSEventModifierFlagCommand | NSEventModifierFlagShift;
    const NSEventModifierFlags cmdOpt = NSEventModifierFlagCommand | NSEventModifierFlagOption;
    const NSEventModifierFlags cmdCtrl = NSEventModifierFlagCommand | NSEventModifierFlagControl;
    NSMenu* mainMenu = [[NSMenu alloc] init];
    NSString* appName = @"Gradient Mesh Studio";

    // --- App menu ---
    NSMenu* appMenu = [self addTopLevelMenuTitled:appName to:mainMenu];
    [appMenu addItemWithTitle:[NSString stringWithFormat:@"About %@", appName] action:@selector(orderFrontStandardAboutPanel:) keyEquivalent:@""];
    [appMenu addItem:[NSMenuItem separatorItem]];
    NSMenuItem* servicesItem = [appMenu addItemWithTitle:@"Services" action:nil keyEquivalent:@""];
    NSMenu* servicesMenu = [[NSMenu alloc] initWithTitle:@"Services"];
    servicesItem.submenu = servicesMenu;
    NSApp.servicesMenu = servicesMenu;
    [appMenu addItem:[NSMenuItem separatorItem]];
    [self addStandardItemTo:appMenu title:[NSString stringWithFormat:@"Hide %@", appName] action:@selector(hide:) key:@"h" modifiers:cmd];
    [self addStandardItemTo:appMenu title:@"Hide Others" action:@selector(hideOtherApplications:) key:@"h" modifiers:cmdOpt];
    [appMenu addItemWithTitle:@"Show All" action:@selector(unhideAllApplications:) keyEquivalent:@""];
    [appMenu addItem:[NSMenuItem separatorItem]];
    [self addStandardItemTo:appMenu title:[NSString stringWithFormat:@"Quit %@", appName] action:@selector(terminate:) key:@"q" modifiers:cmd];

    // --- File menu: Open, Close, Save (the app's only "save" is a preset --
    // there is no document file), Export for the PNG/SVG renderings. ---
    NSMenu* fileMenu = [self addTopLevelMenuTitled:@"File" to:mainMenu];
    [self addItemTo:fileMenu title:@"Open Image…" action:@selector(openImage:) key:@"o"];
    [fileMenu addItem:[NSMenuItem separatorItem]];
    [self addStandardItemTo:fileMenu title:@"Close" action:@selector(performClose:) key:@"w" modifiers:cmd];
    [fileMenu addItem:[NSMenuItem separatorItem]];
    [self addItemTo:fileMenu title:@"Save Preset…" action:@selector(savePreset:) key:@"s"];
    NSMenuItem* exportItem = [fileMenu addItemWithTitle:@"Export" action:nil keyEquivalent:@""];
    NSMenu* exportMenu = [[NSMenu alloc] initWithTitle:@"Export"];
    [self addItemTo:exportMenu title:@"PNG…" action:@selector(exportPNG:) key:@""];
    [self addItemTo:exportMenu title:@"SVG…" action:@selector(exportSVG:) key:@""];
    [self addItemTo:exportMenu title:@"GPU PNG…" action:@selector(exportGPUPNG:) key:@""];
    exportItem.submenu = exportMenu;

    // --- Edit menu ---
    NSMenu* editMenu = [self addTopLevelMenuTitled:@"Edit" to:mainMenu];
    [editMenu addItemWithTitle:@"Undo" action:@selector(undo:) keyEquivalent:@"z"];
    NSMenuItem* redo = [editMenu addItemWithTitle:@"Redo" action:@selector(redo:) keyEquivalent:@"Z"];
    redo.keyEquivalentModifierMask = cmdShift;
    [editMenu addItem:[NSMenuItem separatorItem]];
    [editMenu addItemWithTitle:@"Cut" action:@selector(cut:) keyEquivalent:@"x"];
    [editMenu addItemWithTitle:@"Copy" action:@selector(copy:) keyEquivalent:@"c"];
    [editMenu addItemWithTitle:@"Paste" action:@selector(paste:) keyEquivalent:@"v"];
    [editMenu addItemWithTitle:@"Select All" action:@selector(selectAll:) keyEquivalent:@"a"];

    // --- View menu: what the canvas and window show. Tags 1-5 map to the
    // inspector's checkboxes (see -toggleViewOption:). ---
    NSMenu* viewMenu = [self addTopLevelMenuTitled:@"View" to:mainMenu];
    [self addItemTo:viewMenu title:@"Show Reconstruction" action:@selector(toggleViewOption:) key:@"" modifiers:cmd tag:1];
    [self addItemTo:viewMenu title:@"Render with GPU (OpenGL)" action:@selector(toggleViewOption:) key:@"" modifiers:cmd tag:2];
    [self addItemTo:viewMenu title:@"Show Mesh" action:@selector(toggleViewOption:) key:@"" modifiers:cmd tag:3];
    [self addItemTo:viewMenu title:@"Show Tangents" action:@selector(toggleViewOption:) key:@"" modifiers:cmd tag:4];
    [self addItemTo:viewMenu title:@"Live Mesh Preview" action:@selector(toggleViewOption:) key:@"" modifiers:cmd tag:5];
    [viewMenu addItem:[NSMenuItem separatorItem]];
    [self addItemTo:viewMenu title:@"Hide Inspector" action:@selector(toggleInspector:) key:@"i" modifiers:cmdOpt tag:0];
    [self addStandardItemTo:viewMenu title:@"Hide Toolbar" action:@selector(toggleToolbarShown:) key:@"t" modifiers:cmdOpt];
    [viewMenu addItem:[NSMenuItem separatorItem]];
    [self addStandardItemTo:viewMenu title:@"Enter Full Screen" action:@selector(toggleFullScreen:) key:@"f" modifiers:cmdCtrl];

    // --- Tool menu: the toolbar's tool selector (tag = segment index) plus
    // the commands of the scribble / vector-line tools. ---
    NSMenu* toolMenu = [self addTopLevelMenuTitled:@"Tool" to:mainMenu];
    NSArray* tools = @[@"Boundary", @"Foreground", @"Background", @"Corners", @"Vector Line", @"Edit Mesh"];
    for (NSUInteger i = 0; i < tools.count; ++i)
        [self addItemTo:toolMenu title:tools[i] action:@selector(selectTool:)
                    key:[NSString stringWithFormat:@"%lu", (unsigned long)(i + 1)] modifiers:cmd tag:(NSInteger)i];
    [toolMenu addItem:[NSMenuItem separatorItem]];
    [self addItemTo:toolMenu title:@"Segment" action:@selector(segmentBoundary:) key:@"" modifiers:cmd tag:0];
    [self addItemTo:toolMenu title:@"Clear Scribbles" action:@selector(clearScribbles:) key:@"" modifiers:cmd tag:0];
    [self addItemTo:toolMenu title:@"Clear Last Line" action:@selector(clearLastLine:) key:@"" modifiers:cmd tag:0];

    // --- Mesh menu: the toolbar's Auto Mesh / Build / Optimize / Animate. ---
    NSMenu* meshMenu = [self addTopLevelMenuTitled:@"Mesh" to:mainMenu];
    [self addItemTo:meshMenu title:@"Auto Mesh" action:@selector(autoMesh:) key:@"m" modifiers:cmdShift tag:0];
    [self addItemTo:meshMenu title:@"Build Initial Mesh" action:@selector(buildMesh:) key:@"b" modifiers:cmd tag:0];
    [self addItemTo:meshMenu title:@"Optimize" action:@selector(optimize:) key:@"r" modifiers:cmd tag:0];
    [meshMenu addItem:[NSMenuItem separatorItem]];
    [self addItemTo:meshMenu title:@"Animate Mesh" action:@selector(toggleAnimateMesh:) key:@"a" modifiers:cmdShift tag:0];
    [meshMenu addItem:[NSMenuItem separatorItem]];
    [self addItemTo:meshMenu title:@"Reset Weights to Defaults" action:@selector(resetWeights:) key:@"" modifiers:cmd tag:0];

    // --- Window menu (Close lives in File) ---
    NSMenu* windowMenu = [self addTopLevelMenuTitled:@"Window" to:mainMenu];
    [self addStandardItemTo:windowMenu title:@"Minimize" action:@selector(performMiniaturize:) key:@"m" modifiers:cmd];
    [windowMenu addItemWithTitle:@"Zoom" action:@selector(performZoom:) keyEquivalent:@""];
    [windowMenu addItem:[NSMenuItem separatorItem]];
    [windowMenu addItemWithTitle:@"Bring All to Front" action:@selector(arrangeInFront:) keyEquivalent:@""];
    NSApp.windowsMenu = windowMenu;

    // --- Help menu: no help book yet, but registering the menu lets macOS
    // add its search field, which finds every menu command above. ---
    NSMenu* helpMenu = [self addTopLevelMenuTitled:@"Help" to:mainMenu];
    NSApp.helpMenu = helpMenu;

    NSApp.mainMenu = mainMenu;
}

- (void)applicationDidFinishLaunching:(NSNotification*)notification {
    self.mainWindowController = [[MainWindowController alloc] init];
    [self buildMainMenu];
    [self.mainWindowController showWindow:nil];
    [self.mainWindowController.window makeKeyAndOrderFront:nil];
    [NSApp activateIgnoringOtherApps:YES];
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)sender {
    return YES;
}

@end
