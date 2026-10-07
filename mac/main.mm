#import <Cocoa/Cocoa.h>
#import "AppDelegate.h"

int main(int argc, const char* argv[]) {
    @autoreleasepool {
        // Show widget tooltips immediately instead of after the default ~1 s
        // hover delay. Must be set before the first window is created.
        [[NSUserDefaults standardUserDefaults] setInteger:0 forKey:@"NSInitialToolTipDelay"];
        NSApplication* app = [NSApplication sharedApplication];
        AppDelegate* delegate = [[AppDelegate alloc] init];
        app.delegate = delegate;
        [app setActivationPolicy:NSApplicationActivationPolicyRegular];
        [app run];
    }
    return 0;
}
