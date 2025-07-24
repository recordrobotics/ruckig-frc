export module ui.uimodule;

export namespace ui
{
    export class UIModule
    {
    public:
        virtual void render() = 0;
        virtual ~UIModule() = default;
    };
}