import {Modal, InputNumber, Form} from "antd";
import {useTranslation} from "react-i18next";
import type {LidarIgnoreCorridor} from "../../../types/ros.ts";

interface EditLidarCorridorModalProps {
    corridor: LidarIgnoreCorridor | null;
    busy: boolean;
    onSave: (widthM: number) => void;
    onCancel: () => void;
}

/// Mobile-friendly stand-in for the desktop LidarCorridorsPanel's inline width
/// field: there is no side panel on mobile, so "Edit properties" on a
/// selected ignore line opens this instead. Position/points are still edited
/// by dragging on the map (same DrawControl session as desktop) - only the
/// width needs a control that isn't a drag gesture.
export const EditLidarCorridorModal = ({corridor, busy, onSave, onCancel}: EditLidarCorridorModalProps) => {
    const {t} = useTranslation();
    const [form] = Form.useForm<{widthCm: number}>();

    if (!corridor) return null;

    return (
        <Modal
            open
            title={corridor.name || t('mapLidarCorridors.unnamed', {id: corridor.id ?? ''})}
            okText={t('mapLidarCorridors.save')}
            cancelText={t('mapLidarCorridors.cancel')}
            confirmLoading={busy}
            onCancel={onCancel}
            onOk={() => {
                const widthCm = form.getFieldValue('widthCm') as number;
                onSave(widthCm / 100);
            }}
        >
            <Form form={form} layout="vertical" initialValues={{widthCm: Math.round((corridor.width_m ?? 0.2) * 100)}}>
                <Form.Item name="widthCm" label={t('mapLidarCorridors.widthLabel')}
                    tooltip={t('mapLidarCorridors.widthTooltip')}>
                    <InputNumber min={5} max={100} step={5} precision={0} addonAfter="cm" style={{width: '100%'}} autoFocus/>
                </Form.Item>
            </Form>
        </Modal>
    );
};
